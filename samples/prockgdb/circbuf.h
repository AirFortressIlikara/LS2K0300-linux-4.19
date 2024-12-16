#ifndef __CIRC_BUF_H__
#define __CIRC_BUF_H__
/* Return count in buffer.  */
#define CIRC_CNT(head, tail, size) (((head) - (tail)) & ((size) - 1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
#define CIRC_SPACE(head, tail, size) CIRC_CNT((tail), ((head) + 1), (size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
#define CIRC_CNT_TO_END(head, tail, size)		\
	({int end = (size) - (tail);			\
		int n = ((head) + end) & ((size) - 1);	\
		n < end ? n : end; })

/* Return space available up to the end of the buffer.  */
#define CIRC_SPACE_TO_END(head, tail, size)		\
	({int end = (size) - 1 - (head);		\
		int n = (end + (tail)) & ((size) - 1);	\
		n <= end ? n : end + 1; })


#define circ_empty(circ)		((circ)->head == (circ)->tail)
#define circ_clear(circ)		((circ)->head = (circ)->tail = 0)

#define circ_chars_pending(circ)					\
	(CIRC_CNT((circ)->head, (circ)->tail, CIRCBUF_XMIT_SIZE))

#define circ_chars_free(circ)						\
	(CIRC_SPACE((circ)->head, (circ)->tail, CIRCBUF_XMIT_SIZE))
#endif
