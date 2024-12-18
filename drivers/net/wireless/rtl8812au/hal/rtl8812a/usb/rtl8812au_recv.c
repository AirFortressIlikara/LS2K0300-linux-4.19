/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _RTL8812AU_RECV_C_

//#include <drv_types.h>
#include <rtl8812a_hal.h>


void rtl8812au_init_recvbuf(_adapter *padapter, struct recv_buf *precvbuf)
{

	precvbuf->transfer_len = 0;

	precvbuf->len = 0;

	precvbuf->ref_cnt = 0;

	if (precvbuf->pbuf) {
		precvbuf->pdata = precvbuf->phead = precvbuf->ptail = precvbuf->pbuf;
		precvbuf->pend = precvbuf->pdata + MAX_RECVBUF_SZ;
	}

}

int	rtl8812au_init_recv_priv(_adapter *padapter)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	int	i, res = _SUCCESS;
	struct recv_buf *precvbuf;

#ifdef CONFIG_RECV_THREAD_MODE
	_rtw_init_sema(&precvpriv->recv_sema, 0);//will be removed
	_rtw_init_sema(&precvpriv->terminate_recvthread_sema, 0);//will be removed
#endif

#ifdef PLATFORM_LINUX
	tasklet_init(&precvpriv->recv_tasklet,
		     (void(*)(unsigned long))rtl8812au_recv_tasklet,
		     (unsigned long)padapter);
#endif

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
#ifdef PLATFORM_LINUX
	precvpriv->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (precvpriv->int_in_urb == NULL) {
		res = _FAIL;
		DBG_8192C("alloc_urb for interrupt in endpoint fail !!!!\n");
		goto exit;
	}
#endif
	precvpriv->int_in_buf = rtw_zmalloc(INTERRUPT_MSG_FORMAT_LEN);
	if (precvpriv->int_in_buf == NULL) {
		res = _FAIL;
		DBG_8192C("alloc_mem for interrupt in endpoint fail !!!!\n");
		goto exit;
	}
#endif

	//init recv_buf
	_rtw_init_queue(&precvpriv->free_recv_buf_queue);

#ifdef CONFIG_USE_USB_BUFFER_ALLOC_RX
	_rtw_init_queue(&precvpriv->recv_buf_pending_queue);
#endif	// CONFIG_USE_USB_BUFFER_ALLOC_RX

	precvpriv->pallocated_recv_buf = rtw_zmalloc(NR_RECVBUFF * sizeof(
			struct recv_buf) + 4);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		RT_TRACE(_module_rtl871x_recv_c_, _drv_err_, ("alloc recv_buf fail!\n"));
		goto exit;
	}
	_rtw_memset(precvpriv->pallocated_recv_buf, 0,
		    NR_RECVBUFF * sizeof(struct recv_buf) + 4);

	precvpriv->precv_buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(
				       precvpriv->pallocated_recv_buf), 4);
	//precvpriv->precv_buf = precvpriv->pallocated_recv_buf + 4 -
	//						((uint) (precvpriv->pallocated_recv_buf) &(4-1));


	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		_rtw_init_listhead(&precvbuf->list);

		_rtw_spinlock_init(&precvbuf->recvbuf_lock);

		precvbuf->alloc_sz = MAX_RECVBUF_SZ;

		res = rtw_os_recvbuf_resource_alloc(padapter, precvbuf);
		if (res == _FAIL)
			break;

		precvbuf->ref_cnt = 0;
		precvbuf->adapter = padapter;


		//rtw_list_insert_tail(&precvbuf->list, &(precvpriv->free_recv_buf_queue.queue));

		precvbuf++;

	}

	precvpriv->free_recv_buf_queue_cnt = NR_RECVBUFF;

#ifdef PLATFORM_LINUX

	skb_queue_head_init(&precvpriv->rx_skb_queue);

#ifdef CONFIG_PREALLOC_RECV_SKB
	{
		int i;
		SIZE_PTR tmpaddr = 0;
		SIZE_PTR alignment = 0;
		struct sk_buff *pskb = NULL;

		skb_queue_head_init(&precvpriv->free_recv_skb_queue);

		for (i = 0; i < NR_PREALLOC_RECV_SKB; i++) {

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)) // http://www.mail-archive.com/netdev@vger.kernel.org/msg17214.html
			pskb = __dev_alloc_skb(MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ, GFP_KERNEL);
#else
			pskb = __netdev_alloc_skb(padapter->pnetdev,
						  MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ, GFP_KERNEL);
#endif

			if (pskb) {
				pskb->dev = padapter->pnetdev;

				tmpaddr = (SIZE_PTR)pskb->data;
				alignment = tmpaddr & (RECVBUFF_ALIGN_SZ - 1);
				skb_reserve(pskb, (RECVBUFF_ALIGN_SZ - alignment));

				skb_queue_tail(&precvpriv->free_recv_skb_queue, pskb);
			}

			pskb = NULL;

		}
	}
#endif

#endif

exit:

	return res;

}

void rtl8812au_free_recv_priv(_adapter *padapter)
{
	int	i;
	struct recv_buf	*precvbuf;
	struct recv_priv	*precvpriv = &padapter->recvpriv;

	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		rtw_os_recvbuf_resource_free(padapter, precvbuf);
		precvbuf++;
	}

	if (precvpriv->pallocated_recv_buf)
		rtw_mfree(precvpriv->pallocated_recv_buf,
			  NR_RECVBUFF * sizeof(struct recv_buf) + 4);

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
#ifdef PLATFORM_LINUX
	if (precvpriv->int_in_urb) {
		usb_free_urb(precvpriv->int_in_urb);
	}
#endif//PLATFORM_LINUX

	if (precvpriv->int_in_buf)
		rtw_mfree(precvpriv->int_in_buf, INTERRUPT_MSG_FORMAT_LEN);
#endif//CONFIG_USB_INTERRUPT_IN_PIPE

#ifdef PLATFORM_LINUX

	if (skb_queue_len(&precvpriv->rx_skb_queue)) {
		DBG_8192C(KERN_WARNING "rx_skb_queue not empty\n");
	}

	skb_queue_purge(&precvpriv->rx_skb_queue);

#ifdef CONFIG_PREALLOC_RECV_SKB

	if (skb_queue_len(&precvpriv->free_recv_skb_queue)) {
		DBG_8192C(KERN_WARNING "free_recv_skb_queue not empty, %d\n",
			  skb_queue_len(&precvpriv->free_recv_skb_queue));
	}

	skb_queue_purge(&precvpriv->free_recv_skb_queue);

#endif

#endif

}


