/* eBPF example program:
 *
 * - Creates arraymap in kernel with 4 bytes keys and 8 byte values
 *
 * - Loads eBPF program
 *
 *   The eBPF program accesses the map passed in to store two pieces of
 *   information. The number of invocations of the program, which maps
 *   to the number of packets received, is stored to key 0. Key 1 is
 *   incremented on each iteration by the number of bytes stored in
 *   the skb.
 *
 * - Detaches any eBPF program previously attached to the cgroup
 *
 * - Attaches the new program to a cgroup using BPF_PROG_ATTACH
 *
 * - Every second, reads map[0] and map[1] to see how many bytes and
 *   packets were seen on any socket of tasks in the given cgroup.
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/bpf.h>

#include "libbpf.h"

enum {
	MAP_KEY_PACKETS,
	MAP_KEY_BYTES,
};

static int prog_load(int map_fd)
{
	struct bpf_insn prog[] = {
		BPF_MOV64_REG(BPF_REG_6, BPF_REG_1), /* save r6 so it's not clobbered by BPF_CALL */

		BPF_MOV64_IMM(BPF_REG_0, MAP_KEY_PACKETS), /* r0 = 0 */
		BPF_STX_MEM(BPF_W, BPF_REG_10, BPF_REG_0, -4), /* *(u32 *)(fp - 4) = r0 */
		BPF_MOV64_REG(BPF_REG_2, BPF_REG_10),
		BPF_ALU64_IMM(BPF_ADD, BPF_REG_2, -4), /* r2 = fp - 4 */
		BPF_LD_MAP_FD(BPF_REG_1, map_fd), /* load map fd to r1 */
		BPF_RAW_INSN(BPF_JMP | BPF_CALL, 0, 0, 0, BPF_FUNC_map_lookup_elem),
		BPF_JMP_IMM(BPF_JEQ, BPF_REG_0, 0, 2),
		BPF_MOV64_IMM(BPF_REG_1, 1), /* r1 = 1 */
		BPF_RAW_INSN(BPF_STX | BPF_XADD | BPF_DW, BPF_REG_0, BPF_REG_1, 0, 0), /* xadd r0 += r1 */


		BPF_MOV64_IMM(BPF_REG_0, MAP_KEY_BYTES), /* r0 = 1 */
		BPF_STX_MEM(BPF_W, BPF_REG_10, BPF_REG_0, -4), /* *(u32 *)(fp - 4) = r0 */
		BPF_MOV64_REG(BPF_REG_2, BPF_REG_10),
		BPF_ALU64_IMM(BPF_ADD, BPF_REG_2, -4), /* r2 = fp - 4 */
		BPF_LD_MAP_FD(BPF_REG_1, map_fd),
		BPF_RAW_INSN(BPF_JMP | BPF_CALL, 0, 0, 0, BPF_FUNC_map_lookup_elem),
		BPF_JMP_IMM(BPF_JEQ, BPF_REG_0, 0, 2),
		BPF_LDX_MEM(BPF_W, BPF_REG_1, BPF_REG_6, offsetof(struct __sk_buff, len)), /* r1 = skb->len */
		BPF_RAW_INSN(BPF_STX | BPF_XADD | BPF_DW, BPF_REG_0, BPF_REG_1, 0, 0), /* xadd r0 += r1 */

		BPF_MOV64_IMM(BPF_REG_0, 1), /* r0 = 1 */
		BPF_EXIT_INSN(),
	};

	return bpf_prog_load(BPF_PROG_TYPE_CGROUP_SOCKET_FILTER,
			     prog, sizeof(prog), "GPL", 0);
}

int main(int argc, char **argv)
{
	int cg_fd, map_fd, prog_fd, key, ret;
	long long pkt_cnt, byte_cnt;

	if (argc < 2) {
		printf("Usage: %s <cg-path>\n", argv[0]);
		return EXIT_FAILURE;
	}

	cg_fd = open(argv[1], O_DIRECTORY | O_RDONLY);
	if (cg_fd < 0) {
		printf("Failed to open cgroup path: '%s'\n",
		       strerror(errno));
		return EXIT_FAILURE;
	}

	map_fd = bpf_create_map(BPF_MAP_TYPE_ARRAY,
				sizeof(key), sizeof(byte_cnt),
				256, 0);
	if (map_fd < 0) {
		printf("Failed to create map: '%s'\n", strerror(errno));
		return EXIT_FAILURE;
	}

	prog_fd = prog_load(map_fd);
	printf("Output from kernel verifier:\n%s\n-------\n", bpf_log_buf);

	if (prog_fd < 0) {
		printf("Failed to load prog: '%s'\n", strerror(errno));
		return EXIT_FAILURE;
	}

	ret = bpf_prog_detach(cg_fd, BPF_ATTACH_TYPE_CGROUP_INET_INGRESS);
	printf("bpf_prog_detach() returned '%s' (%d)\n",
	       strerror(errno), errno);

	ret = bpf_prog_attach(prog_fd, cg_fd,
			      BPF_ATTACH_TYPE_CGROUP_INET_INGRESS);
	if (ret < 0) {
		printf("Failed to attach prog to cgroup: '%s'\n",
		       strerror(errno));
		return EXIT_FAILURE;
	}

	while (1) {
		key = MAP_KEY_PACKETS;
		assert(bpf_lookup_elem(map_fd, &key, &pkt_cnt) == 0);

		key = MAP_KEY_BYTES;
		assert(bpf_lookup_elem(map_fd, &key, &byte_cnt) == 0);

		printf("cgroup received %lld packets, %lld bytes\n",
		       pkt_cnt, byte_cnt);
		sleep(1);
	}

	return EXIT_SUCCESS;
}
