/*
 * This is a cgroup controller that acts as container for eBPF programs.
 *
 * Copyright (C) 2016 Daniel Mack
 *
 * This file is subject to the terms and conditions of version 2 of the GNU
 * General Public License.  See the file COPYING in the main directory of the
 * Linux distribution for more details.
 */

#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/cgroup.h>
#include <linux/slab.h>
#include <linux/bpf.h>
#include <linux/filter.h>
#include <net/sock.h>

DEFINE_STATIC_KEY_FALSE(cgroup_bpf_enabled_key);
EXPORT_SYMBOL(cgroup_bpf_enabled_key);

struct bpf_cgroup {
	struct cgroup_subsys_state css;

	/*
	 * Store two sets of bpf_prog pointers, one for programs that are
	 * pinned directly to this css, and one for those that are effective
	 * when this css is accessed.
	 */
	struct bpf_prog *prog[__MAX_BPF_ATTACH_TYPE];
	struct bpf_prog *effective[__MAX_BPF_ATTACH_TYPE];
};

static struct bpf_cgroup *css_bpf(struct cgroup_subsys_state *css)
{
	return container_of(css, struct bpf_cgroup, css);
}

static struct cgroup_subsys_state *
bpf_css_alloc(struct cgroup_subsys_state *parent)
{
	struct bpf_cgroup *bpf;

	bpf = kzalloc(sizeof(*bpf), GFP_KERNEL);
	if (!bpf)
		return ERR_PTR(-ENOMEM);

	if (parent) {
		struct bpf_cgroup *parent_bpf = css_bpf(parent);
		unsigned int type;

		rcu_read_lock();

		for (type = 0; type < __MAX_BPF_ATTACH_TYPE; type++)
			rcu_assign_pointer(bpf->effective[type],
				rcu_dereference(parent_bpf->effective[type]));

		rcu_read_unlock();
	}

	return &bpf->css;
}

static void bpf_css_free(struct cgroup_subsys_state *css)
{
	struct bpf_cgroup *bpf = css_bpf(css);
	unsigned int type;

	for (type = 0; type < __MAX_BPF_ATTACH_TYPE; type++)
		if (bpf->prog[type]) {
			static_branch_dec(&cgroup_bpf_enabled_key);
			bpf_prog_put(bpf->prog[type]);
		}

	kfree(css_bpf(css));
}

static struct cftype bpf_files[] = {
	{ }     /* terminate */
};

struct cgroup_subsys bpf_cgrp_subsys = {
	.css_alloc	= bpf_css_alloc,
	.css_free	= bpf_css_free,
	.dfl_cftypes	= bpf_files,
};

/**
 * __cgroup_bpf_update() - Update the pinned program of a cgroup, and
 *			   propagate the change to descendants
 * @cgrp: The cgroup which descendants to traverse
 * @prog: A new program to pin
 * @type: Type of pinning operation (ingress/egress)
 *
 * Each cgroup has a set of two pointers for bpf programs; one for eBPF
 * programs it owns, and which is effective for execution.
 *
 * If @prog is %NULL, this function attaches a new program to the cgroup and
 * releases the one that is currently attached, if any. @prog is then made
 * the effective program of type @type in that cgroup.
 *
 * If @prog is %NULL, the currently attached program of type @type is released,
 * and, the effective program of the parent cgroup is inherited to @cgrp.
 *
 * Then, the descendants of @cgrp are walked and the effective program for
 * each of them is set to the effective program of @cgrp, unless the
 * descendant has its own program attached, in which case the subbranch is
 * skipped. This ensures that delegated subcgroups with own programs are left
 * untouched.
 *
 * Must be called with cgroup_mutex held.
 */
int __cgroup_bpf_update(struct cgroup *cgrp,
			struct bpf_prog *prog,
			enum bpf_attach_type type)
{
	struct cgroup_subsys_state *css, *pos, *parent;
	struct bpf_prog *old_prog, *effective;
	struct bpf_cgroup *bpf, *parent_bpf;
	int ret = 0;

	rcu_read_lock();

	css = rcu_dereference(cgrp->subsys[bpf_cgrp_id]);
	if (!css) {
		ret = -EINVAL;
		goto exit_unlock;
	}

	bpf = css_bpf(css);
	parent_bpf = css_bpf(css->parent);

	old_prog = xchg(bpf->prog + type, prog);
	if (old_prog)
		bpf_prog_put(old_prog);

	if (prog)
		static_branch_inc(&cgroup_bpf_enabled_key);

	if (old_prog)
		static_branch_dec(&cgroup_bpf_enabled_key);

	effective = (!prog && parent) ?
		rcu_dereference(parent_bpf->effective[type]) : prog;

	css_for_each_descendant_pre(pos, css) {
		struct bpf_cgroup *pos_bpf = css_bpf(pos);

		/* skip the subtree if the descendant has its own program */
		if (pos_bpf->prog[type] && pos != css)
			pos = css_rightmost_descendant(pos);
		else
			rcu_assign_pointer(pos_bpf->effective[type], effective);
	}

exit_unlock:
	rcu_read_unlock();

	return ret;
}

/**
 * cgroup_bpf_run_filter - Run a program for packet filtering
 * @sk: The socken sending or receiving traffic
 * @skb: The skb that is being sent or received
 * @type: The type of program to be exectuted
 *
 * If no socket is passed, or the socket is not of type INET or INET6,
 * this function does nothing and returns 0.
 *
 * The program type passed in via @type must be suitable for network
 * filtering. No further check is performed to assert that.
 *
 * This function will return %-EPERM if any if an attached program was found
 * and if it returned != 1 during execution. In all other cases, 0 is returned.
 */
int cgroup_bpf_run_filter(struct sock *sk, struct sk_buff *skb,
			  enum bpf_attach_type type)
{
	struct cgroup_subsys_state *css;
	struct bpf_cgroup *bpf;
	struct bpf_prog *prog;
	struct cgroup *cgrp;
	int ret = 0;

	if (!sk)
		return 0;

	if (sk->sk_family != AF_INET &&
	    sk->sk_family != AF_INET6)
		return 0;

	cgrp = sock_cgroup_ptr(&sk->sk_cgrp_data);

	rcu_read_lock();

	css = rcu_dereference(cgrp->subsys[bpf_cgrp_id]);
	if (!css)
		goto exit_unlock;

	bpf = css_bpf(css);

	prog = rcu_dereference(bpf->effective[type]);
	if (prog) {
		unsigned int offset = skb->data - skb_mac_header(skb);

		__skb_push(skb, offset);
		ret = bpf_prog_run_clear_cb(prog, skb) == 1 ? 0 : -EPERM;
		__skb_pull(skb, offset);
	}

exit_unlock:
	rcu_read_unlock();

	return ret;
}

