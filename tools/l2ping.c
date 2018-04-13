/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <sys/time.h>
#include <poll.h>
#include <sys/socket.h>

#include <sys/ioctl.h>


#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"



#define for_each_opt(opt, long, short) while ((opt=getopt_long(argc, argv, short ? short:"+", long, NULL)) != -1)

/* Defaults */
static bdaddr_t bdaddr;
static int size    = 44;
static int ident   = 200;
static int delay   = 1;
static int count   = -1;
static int timeout = 10;
static int reverse = 0;
static int verify = 0;
static int rssi_req = 1;

static void helper_arg(int min_num_arg, int max_num_arg, int *argc,
			char ***argv, const char *usage)
{
	*argc -= optind;
	/* too many arguments, but when "max_num_arg < min_num_arg" then no
		 limiting (prefer "max_num_arg=-1" to gen infinity)
	*/
	if ( (*argc > max_num_arg) && (max_num_arg >= min_num_arg ) ) {
		fprintf(stderr, "%s: too many arguments (maximal: %i)\n",
				*argv[0], max_num_arg);
		printf("%s", usage);
		exit(1);
	}

	/* print usage */
	if (*argc < min_num_arg) {
		fprintf(stderr, "%s: too few arguments (minimal: %i)\n",
				*argv[0], min_num_arg);
		printf("%s", usage);
		exit(0);
	}

	*argv += optind;
}

static int find_conn(int s, int dev_id, long arg)
{
	struct hci_conn_list_req *cl;
	struct hci_conn_info *ci;
	int i;

	if (!(cl = malloc(10 * sizeof(*ci) + sizeof(*cl)))) {
		perror("Can't allocate memory");
		exit(1);
	}
	cl->dev_id = dev_id;
	cl->conn_num = 10;
	ci = cl->conn_info;

	if (ioctl(s, HCIGETCONNLIST, (void *) cl)) {
		perror("Can't get connection list");
		exit(1);
	}

	for (i = 0; i < cl->conn_num; i++, ci++)
		if (!bacmp((bdaddr_t *) arg, &ci->bdaddr)) {
			free(cl);
			return 1;
		}

	free(cl);
	return 0;
}





/* Read RSSI */

static struct option rssi_options[] = {
	{ "help",	0, 0, 'h' },
	{ 0, 0, 0, 0 }
};

static const char *rssi_help =
	"Usage:\n"
	"\trssi <bdaddr>\n";

//device id( es: 0 per hci0, 1 per hci1), argc = 2 idk why, svr is the target bdr address
static int cmd_rssi(int dev_id, int argc, char *svr)
{
	struct hci_conn_info_req *cr;
	bdaddr_t bdaddr;
	int8_t rssi;
	int opt, dd;

/*
	for_each_opt(opt, rssi_options, NULL) {
		switch (opt) {
		default:
			printf("%s", rssi_help);
			return;
		}
	}
	helper_arg(1, 1, &argc, &argv, rssi_help);
*/
	str2ba(svr, &bdaddr);

	if (dev_id < 0) {
		dev_id = hci_for_each_dev(HCI_UP, find_conn, (long) &bdaddr);
		if (dev_id < 0) {
			fprintf(stderr, "Not connected.\n");
			exit(1);
		}
	}

	dd = hci_open_dev(dev_id);
	if (dd < 0) {
		perror("HCI device open failed");
		exit(1);
	}

	cr = malloc(sizeof(*cr) + sizeof(struct hci_conn_info));
	if (!cr) {
		perror("Can't allocate memory");
		exit(1);
	}

	bacpy(&cr->bdaddr, &bdaddr);
	cr->type = ACL_LINK;
	if (ioctl(dd, HCIGETCONNINFO, (unsigned long) cr) < 0) {
		perror("Get connection info failed");
		exit(1);
	}

	if (hci_read_rssi(dd, htobs(cr->conn_info->handle), &rssi, 1000) < 0) {
		perror("Read RSSI failed");
		exit(1);
	}

	//printf("RSSI return value: %d\n", rssi);

	free(cr);

	hci_close_dev(dd);

	return rssi;
}



/* Stats */
static int sent_pkt = 0;
static int recv_pkt = 0;

static float tv2fl(struct timeval tv)
{
	return (float)(tv.tv_sec*1000.0) + (float)(tv.tv_usec/1000.0);
}

static void stat(int sig)
{
	int loss = sent_pkt ? (float)((sent_pkt-recv_pkt)/(sent_pkt/100.0)) : 0;
	printf("%d sent, %d received, %d%% loss\n", sent_pkt, recv_pkt, loss);
	exit(0);
}

static void ping(char *svr)
{
	struct sigaction sa;
	struct sockaddr_l2 addr;
	socklen_t optlen;
	unsigned char *send_buf;
	unsigned char *recv_buf;
	char str[18];
	int i, sk, lost;
	int rssi;
	uint8_t id;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = stat;
	sigaction(SIGINT, &sa, NULL);

	send_buf = malloc(L2CAP_CMD_HDR_SIZE + size);
	recv_buf = malloc(L2CAP_CMD_HDR_SIZE + size);
	if (!send_buf || !recv_buf) {
		perror("Can't allocate buffer");
		exit(1);
	}

	/* Create socket */
	sk = socket(PF_BLUETOOTH, SOCK_RAW, BTPROTO_L2CAP);
	if (sk < 0) {
		perror("Can't create socket");
		goto error;
	}

	/* Bind to local address */
	memset(&addr, 0, sizeof(addr));
	addr.l2_family = AF_BLUETOOTH;
	bacpy(&addr.l2_bdaddr, &bdaddr);

	if (bind(sk, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		perror("Can't bind socket");
		goto error;
	}

	/* Connect to remote device */
	memset(&addr, 0, sizeof(addr));
	addr.l2_family = AF_BLUETOOTH;
	str2ba(svr, &addr.l2_bdaddr);

	if (connect(sk, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		perror("Can't connect");
		goto error;
	}

	/* Get local address */
	memset(&addr, 0, sizeof(addr));
	optlen = sizeof(addr);

	if (getsockname(sk, (struct sockaddr *) &addr, &optlen) < 0) {
		perror("Can't get local address");
		goto error;
	}

	ba2str(&addr.l2_bdaddr, str);
	printf("Ping: %s from %s (data size %d) ...\n", svr, str, size);

	/* Initialize send buffer */
	for (i = 0; i < size; i++)
		send_buf[L2CAP_CMD_HDR_SIZE + i] = (i % 40) + 'A';

	id = ident;

	while (count == -1 || count-- > 0) {
		struct timeval tv_send, tv_recv, tv_diff;
		l2cap_cmd_hdr *send_cmd = (l2cap_cmd_hdr *) send_buf;
		l2cap_cmd_hdr *recv_cmd = (l2cap_cmd_hdr *) recv_buf;

		/* Build command header */
		send_cmd->ident = id;
		send_cmd->len   = htobs(size);

		if (reverse)
			send_cmd->code = L2CAP_ECHO_RSP;
		else
			send_cmd->code = L2CAP_ECHO_REQ;

		gettimeofday(&tv_send, NULL);

		/* Send Echo Command */
		if (send(sk, send_buf, L2CAP_CMD_HDR_SIZE + size, 0) <= 0) {
			perror("Send failed");
			goto error;
		}

		/* Wait for Echo Response */
		lost = 0;
		while (1) {
			struct pollfd pf[1];
			int err;

			pf[0].fd = sk;
			pf[0].events = POLLIN;

			if ((err = poll(pf, 1, timeout * 1000)) < 0) {
				perror("Poll failed");
				goto error;
			}

			if (!err) {
				lost = 1;
				break;
			}

			if ((err = recv(sk, recv_buf, L2CAP_CMD_HDR_SIZE + size, 0)) < 0) {
				perror("Recv failed");
				goto error;
			}

			if (!err){
				printf("Disconnected\n");
				goto error;
			}

			recv_cmd->len = btohs(recv_cmd->len);

			/* Check for our id */
			if (recv_cmd->ident != id)
				continue;

			/* Check type */
			if (!reverse && recv_cmd->code == L2CAP_ECHO_RSP)
				break;

			if (recv_cmd->code == L2CAP_COMMAND_REJ) {
				printf("Peer doesn't support Echo packets\n");
				goto error;
			}

		}
		if(rssi_req)
			rssi = cmd_rssi(0,2, svr);
		

		sent_pkt++;

		if (!lost) {
			recv_pkt++;

			gettimeofday(&tv_recv, NULL);
			timersub(&tv_recv, &tv_send, &tv_diff);

			if (verify) {
				/* Check payload length */
				if (recv_cmd->len != size) {
					fprintf(stderr, "Received %d bytes, expected %d\n",
						   recv_cmd->len, size);
					goto error;
				}

				/* Check payload */
				if (memcmp(&send_buf[L2CAP_CMD_HDR_SIZE],
						   &recv_buf[L2CAP_CMD_HDR_SIZE], size)) {
					fprintf(stderr, "Response payload different.\n");
					goto error;
				}
			}

			printf("%d bytes from %s id %d time %.2fms", recv_cmd->len, svr,
				   id - ident, tv2fl(tv_diff));
			if (rssi_req){
				printf(" and RSSI: %d\n", rssi);
			}
			else
				printf("\n");

			//rssi = cmd_rssi(0,2, svr);

			if (delay)
				sleep(delay);
		} else {
			printf("no response from %s: id %d\n", svr, id - ident);
		}

		if (++id > 254)
			id = ident;
	}
	stat(0);
	free(send_buf);
	free(recv_buf);
	return;

error:
	close(sk);
	free(send_buf);
	free(recv_buf);
	exit(1);
}







static void usage(void)
{
	printf("l2ping - L2CAP ping with RSSI\n");
	printf("Works only with device hci0 (default)\n");
	printf("Usage:\n");
	printf("\tl2ping [-i device] [-s size] [-c count] [-t timeout] [-d delay] [-f] [-r] [-v] [-n] <bdaddr>\n");
	printf("\t-f  Flood ping (delay = 0)\n");
	printf("\t-r  Reverse ping\n");
	printf("\t-v  Verify request and response payload\n");
	printf("\t-n  Does not perform RSSI requests\n");
}

int main(int argc, char *argv[])
{
	int opt;

	/* Default options */
	bacpy(&bdaddr, BDADDR_ANY);

	while ((opt=getopt(argc,argv,"i:d:s:c:t:frvn")) != EOF) {
		switch(opt) {
		case 'i':
			if (!strncasecmp(optarg, "hci", 3))
				hci_devba(atoi(optarg + 3), &bdaddr);

			else
				str2ba(optarg, &bdaddr);
			break;

		case 'd':
			delay = atoi(optarg);
			break;

		case 'f':
			/* Kinda flood ping */
			delay = 0;
			break;

		case 'r':
			/* Use responses instead of requests */
			reverse = 1;
			break;

		case 'v':
			verify = 1;
			break;

		case 'c':
			count = atoi(optarg);
			break;

		case 't':
			timeout = atoi(optarg);
			break;

		case 's':
			size = atoi(optarg);
			break;

		case 'n':
			rssi_req = 0;
			break;

		default:
			usage();
			exit(1);
		}
	}

	if (!(argc - optind)) {
		usage();
		exit(1);
	}

	ping(argv[optind]);

	return 0;
}
