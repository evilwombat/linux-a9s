#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/semaphore.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include "ambafs.h"

#define rpdev_name "aipc_vfs"
#define XFR_ARRAY_SIZE     32

extern int *qstat_buf;

struct ambafs_xfr {
	struct completion comp;
	int               refcnt;
	void              (*cb)(void*, struct ambafs_msg *, int);
	void              *priv;
};

static DEFINE_SPINLOCK(xfr_lock);
static struct semaphore      xfr_sem;
static struct rpmsg_channel *rpdev_vfs;
static struct ambafs_xfr     xfr_slot[XFR_ARRAY_SIZE];

/*
 * find a free xfr slot, inc the ref count and return the slot
 */
static struct ambafs_xfr *find_free_xfr(void)
{
	int i;
	unsigned long flags;

	down(&xfr_sem);
	spin_lock_irqsave(&xfr_lock, flags);
	for (i = 0; i < XFR_ARRAY_SIZE; i++) {
		if (!xfr_slot[i].refcnt) {
			xfr_slot[i].refcnt = 1;
			INIT_COMPLETION(xfr_slot[i].comp);
			spin_unlock_irqrestore(&xfr_lock, flags);
			return &xfr_slot[i];
		}
	}
	spin_unlock_irqrestore(&xfr_lock, flags);

	/* FIXME: should wait for a xfr slot becoming available */
	BUG();
	return NULL;
}

/*
 * increase xfr refcount
 */
static void xfr_get(struct ambafs_xfr *xfr)
{
	unsigned long flags;

	spin_lock_irqsave(&xfr_lock, flags);
	xfr->refcnt++;
	spin_unlock_irqrestore(&xfr_lock, flags);
}

/*
 * decrease xfr refcount
 */
static void xfr_put(struct ambafs_xfr *xfr)
{
	unsigned long flags;
	int count;

	spin_lock_irqsave(&xfr_lock, flags);
	count = --xfr->refcnt;
	spin_unlock_irqrestore(&xfr_lock, flags);

	if (!count)
		up(&xfr_sem);
}

/*
 * xfr callback for ambafs_rpsmg_exec
 * copy the incoming msg and wake up the waiting thread
 */
static void exec_cb(void *priv, struct ambafs_msg *msg, int len)
{
	struct ambafs_msg *out_msg = (struct ambafs_msg*)priv;
	struct ambafs_xfr *xfr = (struct ambafs_xfr*) msg->xfr;

	memcpy(out_msg, msg, len);
	complete(&xfr->comp);
}

/*
 * RPMSG channel callback for incoming rpmsg
 */
static void rpmsg_vfs_recv(struct rpmsg_channel *rpdev, void *data, int len,
		void *priv, u32 src)
{
	struct ambafs_msg *msg = (struct ambafs_msg*) data;
	struct ambafs_xfr *xfr = (struct ambafs_xfr*) msg->xfr;

	if (xfr) {
		xfr->cb(xfr->priv, msg, len);
		xfr_put(xfr);
	}
}

static int rpmsg_vfs_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;
	struct rpmsg_ns_msg nsm;

	if (!strcmp(rpdev->id.name, rpdev_name))
		rpdev_vfs = rpdev;
	nsm.addr = rpdev->dst;
	memcpy(nsm.name, rpdev->id.name, RPMSG_NAME_SIZE);
	nsm.flags = 0;

	qstat_buf = (int *) kmalloc(QSTAT_BUFF_SIZE + 0x20, GFP_KERNEL | GFP_DMA);

	rpmsg_send(rpdev, &nsm, sizeof(nsm));

	return ret;
}

static void rpmsg_vfs_remove(struct rpmsg_channel *rpdev)
{
	kfree((void*) qstat_buf);
}

static struct rpmsg_device_id rpmsg_vfs_id_table[] = {
	{ .name	= rpdev_name, },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_vfs_id_table);

static struct rpmsg_driver rpmsg_vfs_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_vfs_id_table,
	.probe		= rpmsg_vfs_probe,
	.callback	= rpmsg_vfs_recv,
	.remove		= rpmsg_vfs_remove,
};

/*
 * send msg and wait on reply
 */
int ambafs_rpmsg_exec(struct ambafs_msg *msg, int len)
{
	struct ambafs_xfr *xfr = find_free_xfr();

	/* set up xfr for the msg */
	xfr->cb = exec_cb;
	xfr->priv = msg;
	msg->xfr = xfr;

	/*
	 * Need lock xfr here, otherwise xfr could be freed in rpmsg_vfs_recv
	 * before wait_for_completion starts to execute.
	 */
	xfr_get(xfr);
	rpmsg_send(rpdev_vfs, msg, len + sizeof(struct ambafs_msg));
	wait_for_completion(&xfr->comp);
	xfr_put(xfr);

	return 0;
}

/*
 * send msg and return immediately
 */
int ambafs_rpmsg_send(struct ambafs_msg *msg, int len,
		void (*cb)(void*, struct ambafs_msg *, int), void* priv)
{
	if (cb) {
		struct ambafs_xfr *xfr = find_free_xfr();
		xfr->cb = cb;
		xfr->priv = priv;
		msg->xfr = xfr;
	} else {
		msg->xfr = NULL;
	}

	rpmsg_send(rpdev_vfs, msg, len + sizeof(struct ambafs_msg));
	return 0;
}

int __init ambafs_rpmsg_init(void)
{
	int i;

	sema_init(&xfr_sem, XFR_ARRAY_SIZE);
	for (i = 0; i < XFR_ARRAY_SIZE; i++) {
		xfr_slot[i].refcnt = 0;
		init_completion(&xfr_slot[i].comp);
	}

	return register_rpmsg_driver(&rpmsg_vfs_driver);
}

void __exit ambafs_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_vfs_driver);
}
