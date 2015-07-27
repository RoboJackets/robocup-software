#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                                                                                 */
/*---------------------------------------------------------------------------*/
#define IPERF_SERVER_PORT 5001

/*---------------------------------------------------------------------------*/
/* local data                                                                                                                    */
/*---------------------------------------------------------------------------*/
#if LWIP_TCP

static struct tcp_pcb *iperf_pcb;

enum iperfserver_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

struct iperfserver_state
{
  u8_t state;
  u8_t retries;
  struct tcp_pcb *pcb;
  /* pbuf (chain) to recycle */
  struct pbuf *p;
};

/*---------------------------------------------------------------------------*/
/* local functions                                                                                                              */
/*---------------------------------------------------------------------------*/
static err_t iperfserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t iperfserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void iperfserver_error(void *arg, err_t err);
static err_t iperfserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void iperfserver_send(struct tcp_pcb *tpcb, struct iperfserver_state *es);
static void iperfserver_close(struct tcp_pcb *tpcb, struct iperfserver_state *es);

void
iperf_server_init(void)
{
  iperf_pcb = tcp_new();
  if (iperf_pcb != NULL)
  {
    err_t err;

    err = tcp_bind(iperf_pcb, IP_ADDR_ANY, IPERF_SERVER_PORT);
    if (err == ERR_OK)
    {
      iperf_pcb = tcp_listen(iperf_pcb);
      tcp_accept(iperf_pcb, iperfserver_accept);
    }
    else 
    {
      /* abort? output diagnostic? */
    }
  }
  else
  {
    /* abort? output diagnostic? */
  }
}


err_t
iperfserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct iperfserver_state *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* commonly observed practive to call tcp_setprio(), why? */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  es = (struct iperfserver_state *)mem_malloc(sizeof(struct iperfserver_state));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->retries = 0;
    es->p = NULL;
    /* pass newly allocated es to our callbacks */
    tcp_arg(newpcb, es);
    tcp_recv(newpcb, iperfserver_recv);
    tcp_err(newpcb, iperfserver_error);
  
    ret_err = ERR_OK;
  }
  else
  {
    ret_err = ERR_MEM;
  }
  return ret_err;  
}

err_t
iperfserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct iperfserver_state *es;
  err_t ret_err;
  u16_t plen;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  es = (struct iperfserver_state *)arg;
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    iperfserver_close(tpcb, es);    
    ret_err = ERR_OK;   
  }
  else if(err != ERR_OK)
  {
    /* cleanup, for unkown reason */
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    es->p = p;
    plen = es->p->len;
    pbuf_free(es->p);   //receive the package and discard it silently for testing reception bandwidth
    tcp_recved(tpcb, plen); 
   
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unkown es->state, trash data  */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

void
iperfserver_error(void *arg, err_t err)
{
  struct iperfserver_state *es;

  LWIP_UNUSED_ARG(err);

  es = (struct iperfserver_state *)arg;
  if (es != NULL)
  {
    mem_free(es);
  }
}

err_t
iperfserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct iperfserver_state *es;

  LWIP_UNUSED_ARG(len);

  es = (struct iperfserver_state *)arg;
  es->retries = 0;
  
  if(es->p != NULL)
  {
    /* still got pbufs to send */
    tcp_sent(tpcb, iperfserver_sent);
    iperfserver_send(tpcb, es);
  }
  else
  {
    /* no more pbufs to send */
    if(es->state == ES_CLOSING)
    {
      iperfserver_close(tpcb, es);
    }
  }
  return ERR_OK;
}

void
iperfserver_send(struct tcp_pcb *tpcb, struct iperfserver_state *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
  while ((wr_err == ERR_OK) &&
         (es->p != NULL) && 
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
  ptr = es->p;

  /* enqueue data for transmission */
  wr_err = tcp_write(tpcb, ptr->payload, ptr->len, TCP_WRITE_FLAG_COPY);
  if (wr_err == ERR_OK)
  {
     u16_t plen;
      u8_t freed;

     plen = ptr->len;
     /* continue with next pbuf in chain (if any) */
     es->p = ptr->next;
     if(es->p != NULL)
     {
       /* new reference! */
       pbuf_ref(es->p);
     }
     /* chop first pbuf from chain */
      do
      {
        /* try hard to free pbuf */
        freed = pbuf_free(ptr);
      }
      while(freed == 0);
     /* we can read more data now */
     tcp_recved(tpcb, plen);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}

void
iperfserver_close(struct tcp_pcb *tpcb, struct iperfserver_state *es)
{
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
 
  
  if (es != NULL)
  {
    mem_free(es);
  }  
  tcp_close(tpcb);
}

#endif /* LWIP_TCP */
/*****************************************************************************/
/* END OF FILE */

