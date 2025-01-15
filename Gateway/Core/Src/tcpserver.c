#include "lwip/opt.h"

#include "lwip/api.h"
#include "lwip/sys.h"

#include "tcpserver.h"
#include "string.h"

#include "main.h"

static struct netconn *conn, *newconn;
static struct netbuf *buf;
static ip_addr_t *addr;
static unsigned short port;
char msg[100];
char smsg[200];

char frame[50];

int convert_char_to_hex(char c){
	if (c <= 'F' && c >= 'A')
	{
		if(c == 'A') return 10; else if (c == 'B') return 11; else if (c == 'C') return 12; else if (c == 'D') return 13; else if (c == 'E') return 14; else if (c == 'F') return 15;
	}
	if (c <= '9' && c >= 0)
		return c - '0';
}


/**** Send RESPONSE every time the client sends some data ******/
static void tcp_thread(void *arg)
{
	err_t err, accept_err, recv_error;

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn!=NULL)
	{
		/* Bind connection to the port number 7. */
		err = netconn_bind(conn, IP_ADDR_ANY, 7);

		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);

			while (1)
			{
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);

				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{

					/* receive the data from the client */
					while (netconn_recv(newconn, &buf) == ERR_OK)
					{
						/* Extrct the address and port in case they are required */
						addr = netbuf_fromaddr(buf);  // get the address of the client
						port = netbuf_fromport(buf);  // get the Port of the client

						/* If there is some data remaining to be sent, the following process will continue */
						do
						{

							strncpy (msg, buf->p->payload, buf->p->len);   // get the message from the client

							strcpy(frame, msg);

							int len = sprintf (smsg, "\n%s", frame);
							netconn_write(newconn, smsg, len, NETCONN_COPY);  // send the message back to the client

							int index = 0;

							for(int i = 0; i < 16 - 1; i += 2)
							{
								char first = frame[i];
								char second = frame[i + 1];

								int first_integer = convert_char_to_hex(first);
								int second_integer = convert_char_to_hex(second);

								request_array_rx[index++] = 16 * first_integer + second_integer;
							}

							// Or modify the message received, so that we can send it back to the client

							len = sprintf (smsg, "\nREQ:");
							netconn_write(newconn, smsg, len, NETCONN_COPY);

							for(int i = 0; i < 8; i++)
							{
								if(request_array_rx[i] > 0x10) {
									int len = sprintf (smsg, "%x", request_array_rx[i]);
									netconn_write(newconn, smsg, len, NETCONN_COPY);
								}
								else {
									int len = sprintf (smsg, "0%x", request_array_rx[i]);
									netconn_write(newconn, smsg, len, NETCONN_COPY);
								}
							}

							len = sprintf (smsg, "\nRES:");
							netconn_write(newconn, smsg, len, NETCONN_COPY);  // send the message back to the client

							for(int i = 0; i < 8; i++)
							{
								if(response_array_rx[i] > 0x10) {
									int len = sprintf (smsg, "%x", response_array_rx[i]);
									netconn_write(newconn, smsg, len, NETCONN_COPY);  // send the message back to the client
								}
								else {
									int len = sprintf (smsg, "0%x", response_array_rx[i]);
									netconn_write(newconn, smsg, len, NETCONN_COPY);  // send the message back to the client
								}
							}

							len = sprintf (smsg, " \n");
							netconn_write(newconn, smsg, len, NETCONN_COPY);



							memset (msg, '\0', 100);  // clear the buffer


						}
						while (netbuf_next(buf) >0);

						netbuf_delete(buf);
					}

					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		}
		else
		{
			netconn_delete(conn);
		}
	}
}


void tcpserver_init(void)
{
  sys_thread_new("tcp_thread", tcp_thread, NULL, DEFAULT_THREAD_STACKSIZE,osPriorityNormal);
}
