#include <linux/device.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/sched.h>
#include <linux/serdev.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
#include <linux/can/platform/mcp251x.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/version.h>

#define CSM330A_DEV_VERSION "1.0"
#define CSM330A_DEV_NAME "ZLGCSM330A"

#define UART_TRANSFER_BUF_LEN   256
#define SET_CSM330_FLAG 1
#define CAN_FRAME_MAX_DATA_LEN  8
#define TX_ECHO_SKB_MAX 1
#define DEVICE_NAME "csm330"

struct csm330_priv {
    struct can_priv    can;
    struct net_device *net;
    struct serdev_device *serdev;
    int frame_num;
    int busy;
    int data_len;
    int bitrate;
    unsigned int RST;
    unsigned int CFG;
    unsigned int flags;
    unsigned char *rx_buf;
    unsigned char *tx_buf;
    uint8_t check_buf[72];
    struct mutex csm_lock;
    int tx_len;
    struct workqueue_struct *wq;
    struct work_struct tx_work;
    struct timer_list timer;

};

#define UART_BPS_DEF    (1000000)  /* default uart bps connected to 330A */

static u32 bitrate_support[15]={5000,10000,20000,40000,50000,80000,100000,125000,200000,250000,400000,
                        500000,666000,800000,1000000};
static uint32_t uart_bps_support[20] = { 
         0,  115200,   57600,   38400,  19200,  14400,   9600,   4800,
      2400,    1200,     600,     300, 128000, 230400, 256000, 460800,
    921600, 1000000, 1500000, 2000000,
};

static int csm330_do_set_bittiming(struct net_device *net)
{
    struct csm330_priv *priv = netdev_priv(net);
    u32 bitrate = priv->can.bittiming.bitrate;
    if((bitrate != 5000) && (bitrate != 10000) && (bitrate != 20000) && (bitrate != 40000)
        && (bitrate != 50000) && (bitrate != 80000) && (bitrate != 100000) && (bitrate != 125000)
         && (bitrate != 200000) && (bitrate != 250000) && (bitrate != 400000) && (bitrate != 500000)
          && (bitrate != 666000) && (bitrate != 800000) && (bitrate != 1000000))
    {
        printk("bitrate %d is not support\n",bitrate);
    }else
        printk("set bitrate is %d\n",priv->can.bittiming.bitrate);
    
    return 0;
}

static const struct can_bittiming_const csm330_bittiming_const = {
    .name = DEVICE_NAME,
    .tseg1_min = 4,
    .tseg1_max = 16,
    .tseg2_min = 2,
    .tseg2_max = 8,
    .sjw_max = 4,
    .brp_min = 1,
    .brp_max = 256,
    .brp_inc = 1,
};

static int csm330_do_set_mode(struct net_device *net, enum can_mode mode)
{
    return 0;
}

int check_csm330(struct net_device *net)
{
    struct csm330_priv *priv = netdev_priv(net);
    struct serdev_device *serdev = priv->serdev;
    unsigned char last_byte = 0;
    int i = 0;
 
    uint8_t setup_cmd[10] = {0xF7,0xF8,0x02,0x04,0x0A,0x15,0x12,0x03,0};
    //dev_info(&serdev->dev, "check_csm330 debug 1\n");
    serdev_device_set_baudrate(serdev, 9600);
    
        
    for(i = 0; i < 8; i++){
        last_byte ^= setup_cmd[i]; 
    }
    setup_cmd[8] = last_byte;
    
    gpio_set_value(priv->RST,1);
    gpio_set_value(priv->CFG,1);
    msleep(20);
    gpio_set_value(priv->CFG,0);
    msleep(20);
    gpio_set_value(priv->RST,0);
    msleep(20);
    gpio_set_value(priv->RST,1);
    msleep(400);
    memset(priv->check_buf,0,8);
    priv->flags = SET_CSM330_FLAG;
    serdev_device_write_buf(serdev,setup_cmd,9);

    msleep(80);
    gpio_set_value(priv->CFG,1);
    msleep(20);
    gpio_set_value(priv->RST,0);
    msleep(20);
    gpio_set_value(priv->RST,1);
    msleep(400);

#if 0
    dev_err(&serdev->dev, "[0-3]: %02x, %02x, %02x, %02x, flags: %d\r\n",   \
            priv->check_buf[0], priv->check_buf[1], priv->check_buf[2], priv->check_buf[3], priv->flags);
#endif

    if(!priv->flags || priv->check_buf[0]==0xF7){
        if(priv->check_buf[0]==0xF7 && priv->check_buf[1]==0xF8 && 
           priv->check_buf[2]==0x02 && priv->check_buf[3]==0x13){
            dev_info(&serdev->dev,"csm330 setup succeed\n");
            return 0;
        }else{
            dev_info(&serdev->dev,"csm330 setup data failed \n");
            return -1;
        }
        priv->flags=0;
    }else{
        priv->flags=0;
        dev_info(&serdev->dev,"csm330 setup flags failed\n");
        return -1;
    }
}

#if 0
static int csm330_rdcfg(struct net_device *net, int cfg_bps)
{
    int i   = 0;
    uint8_t last_byte   = 0;
    uint8_t rdcfg_cmd[] = { 0xf7, 0xf8, 0x03, 0x00, 0x0c };

    struct csm330_priv   *priv   = NULL;
    struct serdev_device *serdev = NULL;

    if (net == NULL) {
        return -EINVAL;
    }

    for(i = 0; i < 4; i++){
        last_byte ^= rdcfg_cmd[i]; 
    }
    rdcfg_cmd[4] = last_byte;

    priv   = netdev_priv(net);
    serdev = priv->serdev;
    serdev_device_set_baudrate(serdev, 9600);

    /* readback 330A configure */
    gpio_set_value(priv->RST,1);
    gpio_set_value(priv->CFG,1);
    msleep(20);
    gpio_set_value(priv->CFG,0);
    msleep(20);
    gpio_set_value(priv->RST,0);
    msleep(20);
    gpio_set_value(priv->RST,1);
    msleep(400);
    
    priv->flags = SET_CSM330_FLAG;
    serdev_device_write_buf(serdev, rdcfg_cmd, sizeof(rdcfg_cmd));

    msleep(200);
    gpio_set_value(priv->CFG,1);
    msleep(20);
    gpio_set_value(priv->RST,0);
    msleep(20);
    gpio_set_value(priv->RST,1);
    msleep(400);

    priv->bitrate = serdev_device_set_baudrate(serdev, cfg_bps);

#if 0
    if (!priv->flags || priv->check_buf[0]==0xF7) {
        for (i = 0; i < 72; i++) {
            printk(KERN_ERR "read cfg %02x: %#x\r\n", i, priv->check_buf[i]);
        }
    } else {
        priv->flags = 0;
        return -1;
    }
#endif

    return 0;
}
#endif

int csm330_setup(struct net_device *net)
{
    struct csm330_priv *priv = netdev_priv(net);
    struct serdev_device *serdev = priv->serdev;
    int i = 0;
    unsigned char last_byte = 0;
    /* default uart: 115200 */
    uint8_t setup_cmd[66] = {0xF7,0xF8,0x01,0x3C,
                             0x01,0x08,0x01,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,   /*  0 ... 15 */
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 16 ... 31 */
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x02,0x00,0x00,0x00,   /* 32 ... 47 */
                             0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x1A,0x0F,0x05,0x01,0x01,0x24};                 /* 48 ... 60 */
    u32 bitrate = priv->can.bittiming.bitrate;
    priv->bitrate = 9600;

    for(i = 0; i < 15; i++){
        if(bitrate_support[i] == bitrate)
            break;
    }
    if(i == 15)
        return -1;
    setup_cmd[8] = i+1;

    /* check uart bps */
    for (i = 1; i < sizeof(uart_bps_support) / sizeof(uart_bps_support[0]); i++) {
        if (uart_bps_support[i] == UART_BPS_DEF) {
            break;
        }
    }
    if (i >= sizeof(uart_bps_support) / sizeof(uart_bps_support[0])) {
        return -EINVAL;
    }
    setup_cmd[4] = i;
    dev_info(&serdev->dev, "uart bps: %d/%#x, can bps: %d/%#x\r\n", UART_BPS_DEF, setup_cmd[4], bitrate, setup_cmd[8]);

    for(i = 0; i < 64; i++){
        last_byte ^= setup_cmd[i]; 
    }
    
    priv->bitrate = serdev_device_set_baudrate(serdev, priv->bitrate);
    
    setup_cmd[64] = last_byte;
    
    gpio_set_value(priv->RST,1);
    gpio_set_value(priv->CFG,1);
    msleep(20);
    gpio_set_value(priv->CFG,0);
    msleep(20);
    gpio_set_value(priv->RST,0);
    msleep(20);
    gpio_set_value(priv->RST,1);
    msleep(400);
    
    memset(priv->check_buf,0,8);
    priv->flags = SET_CSM330_FLAG;
    serdev_device_write_buf(serdev,setup_cmd,sizeof(setup_cmd));

    msleep(200);
    gpio_set_value(priv->CFG,1);
    msleep(20);
    gpio_set_value(priv->RST,0);
    msleep(20);
    gpio_set_value(priv->RST,1);
    msleep(400);
    
    if(!priv->flags || priv->check_buf[0]==0xF7){
        if(priv->check_buf[0]==0xF7 && priv->check_buf[1]==0xF8 && 
           priv->check_buf[2]==0x01 && priv->check_buf[3]==0x13){
            priv->bitrate = serdev_device_set_baudrate(serdev, uart_bps_support[setup_cmd[4]]);
            return 0;
        }else{
            printk("csm330 setup data failed\n");
            return -1;
        }
        priv->flags=0;
    }else{
        priv->flags=0;
        printk("csm330 setup flags failed\n");
        return -1;
    }
    
}

static int
csm330_tty_receive(struct serdev_device *serdev, const unsigned char *data, size_t count)
{

    struct csm330_priv *priv = serdev_device_get_drvdata(serdev);
    struct net_device *net = priv->net;
    struct can_frame *frame;
    struct sk_buff *skb;
    
    unsigned int frame_size, len = 0;

#if 0
    int i;
    for(i=0;i<count;i++) {
        dev_info(&serdev->dev,"rx buf = %02x\n", data[i]);
    }
#endif

    mutex_lock(&priv->csm_lock);
    //dev_info(&serdev->dev,"flages =%d\n", priv->flags);
    //此处返回0是防止数据缓存数据量过小
    if (count < 5 && priv->flags){
        mutex_unlock(&priv->csm_lock);
        return 0;
    }
    //此处返回count是防止数据缓存数据过大导致缓存区溢出、系统崩溃，根据实质情况自行加减
    if(count > 4000){
        //dev_err(&serdev->dev,"csm330 data too much count = %d\n", count);
        mutex_unlock(&priv->csm_lock);
        return count;
    }

    //up 和 check 时候进行不同的数据处理
    if(priv->flags){ 
        memset(priv->check_buf,0,8);
        if(count > 8){
            memcpy(priv->check_buf, data, 8);
            len = 8;
        }else{
            memcpy(priv->check_buf, data, count);
            len = count;
        }
        priv->flags = 0;
    }else{
        //can 帧数据处理
        memset(priv->rx_buf,0,UART_TRANSFER_BUF_LEN);
        if(count > UART_TRANSFER_BUF_LEN){
            memcpy(priv->rx_buf, data, UART_TRANSFER_BUF_LEN);
        }else{
            memcpy(priv->rx_buf, data, count);
        }

        while(1){
            if(priv->rx_buf[len] == 0x40){
                frame_size = priv->rx_buf[len+1];
//                dev_info(&serdev->dev, "count: %d, frame_size: %d, tail: %#x\r\n", count, frame_size, priv->rx_buf[len+frame_size + 2]);
                if(priv->rx_buf[len+frame_size + 2] != 0x1A){
                    if ((count + len) < len + frame_size + 2) {
                        break;
                    }

                    net->stats.rx_errors++;
                    len += frame_size + 3;
                    continue;
                }
                if((priv->rx_buf[len+2] != 0) && (priv->rx_buf[len+2] != (unsigned char )0x08)){
                    len += (frame_size + 3);
                    net->stats.rx_errors++;
                    continue;
                }
                skb = alloc_can_skb(priv->net, &frame);
                if (!skb) {
                    dev_err(&serdev->dev, "cannot allocate RX skb\n");
                    priv->net->stats.rx_dropped++;
                    len += (frame_size + 3);
                    continue;
                }
                if(priv->rx_buf[len+2] == 0x00){
                    frame->can_id = priv->rx_buf[len+3] << 8;
                    frame->can_id |= priv->rx_buf[len+4];
                    frame->can_dlc = priv->rx_buf[len+1] - 3;
                    memcpy(frame->data, priv->rx_buf+len+5, frame->can_dlc);
                }else{
                    frame->can_id = CAN_EFF_FLAG;
                    frame->can_id |= priv->rx_buf[len+3] << 24;
                    frame->can_id |= priv->rx_buf[len+4] << 16;
                    frame->can_id |= priv->rx_buf[len+5] << 8;
                    frame->can_id |= priv->rx_buf[len+6];
                    frame->can_dlc = priv->rx_buf[len+1] - 5;
                    memcpy(frame->data, priv->rx_buf+len+7, frame->can_dlc);
                }
                len += (frame_size + 3);
                priv->net->stats.rx_packets++;
                priv->net->stats.rx_bytes += frame->can_dlc;
                netif_rx_ni(skb);
            }else{
                len++;
            }
            //数据不够一帧时，先不处理
            if((count-len) < 14 )
                break;
            if(len >= count || len >= 140 ){
                //printk("count = %d\n", count);
                if(len > count){
                    len = count;
                }
                break;  
            }
        }
    }
    mutex_unlock(&priv->csm_lock);
    return len;
}

#if 0
static void csm330_tty_wakeup(struct serdev_device *serdev)
{
    struct csm330_priv *priv = serdev_device_get_drvdata(serdev);

    schedule_work(&priv->tx_work);
}
#endif

static void csm330_tx_work_handler(struct work_struct *ws)
{
    struct csm330_priv *priv = container_of(ws, struct csm330_priv,
                         tx_work);
    struct serdev_device *serdev = priv->serdev;
    int time = ((2*10)/priv->bitrate)+1;
    mutex_lock(&priv->csm_lock);
    priv->busy = 1;

#if 0
    do {
        int i = 0;

        for(i = 0; i < priv->data_len; i++) {
            dev_info(&serdev->dev, "tx buf =%x\r\n", priv->tx_buf[i]);
        }
    } while (0);
#endif

    serdev_device_write_buf(serdev,priv->tx_buf,priv->data_len);
    udelay(time);
    memset(priv->tx_buf,0,UART_TRANSFER_BUF_LEN);
    priv->data_len = 0;
    priv->frame_num = 0;
    priv->busy = 0;
    mutex_unlock(&priv->csm_lock);
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
static void send_data_start(struct timer_list *t)
{
    struct csm330_priv *priv = from_timer(priv, t,timer);
#else
static void send_data_start(unsigned long arg)
{
    struct csm330_priv *priv = netdev_priv((struct net_device *)(arg));
#endif
    queue_work(priv->wq, &priv->tx_work);
}

static int csm330_open(struct net_device *net)
{
    struct csm330_priv *priv = netdev_priv(net);
    struct serdev_device *serdev = priv->serdev ;
//  unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW;
    int ret;

    ret = serdev_device_open(serdev);
    if (ret) {
        dev_err(&serdev->dev, "Unable to open device %s\n",net->name);
        return ret;
    }

    ret = open_candev(net);
    if (ret) {
        serdev_device_close(serdev);
        dev_err(&serdev->dev, "unable to set initial baudrate!\n");
        return ret;
    }
    
    //csm330_setup(net);

    ret = csm330_setup(net);
    if(ret){
        dev_err(&serdev->dev,"setup error\n");
        serdev_device_close(serdev);
        close_candev(net);
        return ret;
    }
//    csm330_rdcfg(net, UART_BPS_DEF);    /* for debug CAN 1Mbps */

    priv->tx_len = 0;
    
    priv->wq = create_singlethread_workqueue("csm330_wq");
    INIT_WORK(&priv->tx_work, csm330_tx_work_handler);
    netif_wake_queue(net);
    
    return ret;
}

static int csm330_stop(struct net_device *net)
{
    struct csm330_priv *priv = netdev_priv(net);
    struct serdev_device *serdev = priv->serdev;
    //free_irq(spi->irq, priv);
    destroy_workqueue(priv->wq);
    serdev_device_close(serdev);
    close_candev(net);
    return 0;
}

static netdev_tx_t csm330_hard_start_xmit(struct sk_buff *skb,
                       struct net_device *net)
{
    struct csm330_priv *priv = netdev_priv(net);
    
    struct can_frame *frame;
    unsigned int id, exide;
    
    unsigned char buf[17] = {0};
    if (mutex_trylock(&priv->csm_lock) != 1) {
    //  dev_warn(&spi->dev, "hard_xmit called while tx busy\n");
        return NETDEV_TX_BUSY;
    }
//  frame = (struct can_frame *)skb->data;
    
    if (can_dropped_invalid_skb(net, skb))
        return NETDEV_TX_OK;
    netif_stop_queue(net);
    can_put_echo_skb(skb, net, 0);

    frame = (struct can_frame *)skb->data;
    if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
        frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
    
    exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
    
    buf[0] = 0x40;
    
    if (exide){
        buf[2] = 0x08;
        id = frame->can_id & CAN_EFF_MASK;  /* Extended ID */
        buf[3] = (id & 0xFF000000) >> 24;
        buf[4] = (id & 0xFF0000) >> 16;
        buf[5] = (id & 0xFF00) >> 8;
        buf[6] = id & 0xFF;
    //  strncpy(&buf[7],frame->data,frame->can_dlc);
    //  for(i = 0;i < frame->can_dlc;i ++ )
    //      buf[7 + i] = frame->data[i];
        memcpy(buf+7, frame->data, frame->can_dlc);
        buf[6+frame->can_dlc+1] = 0x1A;
        buf[1] = 6+frame->can_dlc-1;

    }
    else{
        buf[2] = 0x0;
        id = frame->can_id & CAN_SFF_MASK; /* Standard ID */
        buf[3] = (id & 0xFF00) >> 8;
        buf[4] = id & 0xFF;
    //  strncpy(&buf[5],frame->data,frame->can_dlc);
    //  for(i = 0;i < frame->can_dlc;i ++ )
    //      buf[5 + i] = frame->data[i];
        memcpy(buf+5, frame->data, frame->can_dlc);
        buf[4+frame->can_dlc+1] = 0x1A;
        buf[1] = 4+frame->can_dlc-1;
    
    }
    
    memcpy((priv->tx_buf) + (priv->data_len) , buf, buf[1] +3);
    priv->data_len += (buf[1] +3);
    priv->frame_num ++;
    mutex_unlock(&priv->csm_lock);
    net->stats.tx_packets++;
    net->stats.tx_bytes += frame->can_dlc;
    can_get_echo_skb(net,0);

    
    if(priv->frame_num <= 6){
        mod_timer(&(priv->timer),jiffies + (HZ/1000) * 10 );
    }
    else{
        del_timer(&(priv->timer));
        queue_work(priv->wq, &priv->tx_work);
    }

    netif_wake_queue(net);

    return NETDEV_TX_OK;
}

static const struct net_device_ops csm330_netdev_ops = {
    .ndo_open = csm330_open,
    .ndo_stop = csm330_stop,
    .ndo_start_xmit = csm330_hard_start_xmit,
};

static struct serdev_device_ops csm330_serdev_ops = {
    .receive_buf = csm330_tty_receive,
//  .write_wakeup = csm330_tty_wakeup,
};

static const struct of_device_id csm330_uart_of_match[] = {
    {
     .compatible    = "zhiyuan,csm330",
    },
    {}
};
MODULE_DEVICE_TABLE(of, csm330_uart_of_match);

static int csm330_uart_probe(struct serdev_device *serdev)
{
    struct net_device *net;
    struct csm330_priv *priv;
    int ret = 0, i;
    
    net = alloc_candev(sizeof(struct csm330_priv), TX_ECHO_SKB_MAX);
    
    if(NULL == net)
        return -ENOMEM;
    
    net->netdev_ops = &csm330_netdev_ops;
    net->flags |= IFF_ECHO;
    priv = netdev_priv(net);
    priv->can.bittiming_const = &csm330_bittiming_const;        //csm330波特率可以直接通过UART设置，此处的bittiming和时钟设置
    priv->can.clock.freq = 24000000;                            //只是为了可以通过ip link设置波特率
    priv->can.do_set_mode = csm330_do_set_mode;   
    priv->can.do_set_bittiming = csm330_do_set_bittiming;
    priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK;
    priv->net = net;
    priv->busy = 0;
    priv->frame_num = 0;
    priv->data_len = 0;
    priv->flags = 0;
    priv->bitrate=9600;
    priv->RST = of_get_gpio(serdev->dev.of_node,0);
    priv->CFG = of_get_gpio(serdev->dev.of_node,1);
    
    priv-> rx_buf = devm_kzalloc(&serdev->dev, UART_TRANSFER_BUF_LEN, GFP_KERNEL);
    if (!priv->rx_buf) {
        ret = -ENOMEM;
    }
    
    priv-> tx_buf = devm_kzalloc(&serdev->dev, UART_TRANSFER_BUF_LEN, GFP_KERNEL);
    if (!priv->tx_buf) {
        ret = -ENOMEM;
    }

    if (!gpio_is_valid(priv->RST) || !gpio_is_valid(priv->CFG)) {
        dev_err(&serdev->dev,"invalid GPIO pins, CTL0=%d/CTL1=%d\n", priv->RST, priv->CFG);
        goto out_free;
    }
    ret = devm_gpio_request_one(&serdev->dev, priv->CFG, GPIOF_OUT_INIT_LOW,"CFG");
    if (ret) {
        dev_err(&serdev->dev,"gpio_request CFG error: %d\r\n", ret);
//        goto out_free;
    }
    ret = devm_gpio_request_one(&serdev->dev, priv->RST,GPIOF_OUT_INIT_LOW, "RST");
    if (ret) {
        dev_err(&serdev->dev,"gpio_request RST error: %d\r\n", ret);
//        goto out_free;
    }
    
    priv->serdev = serdev;
    
    serdev_device_set_drvdata(serdev, priv);
    serdev_device_set_client_ops(serdev, &csm330_serdev_ops);
    
    ret = serdev_device_open(serdev);
    if (ret) {
        dev_err(&serdev->dev, "Unable to open device %s\n",net->name);
        goto out_free;
    }
    serdev_device_set_flow_control(serdev, false);  

    priv->bitrate = serdev_device_set_baudrate(serdev, priv->bitrate);
    dev_info(&serdev->dev, "Using baudrate: %u\n", priv->bitrate);
    
    for(i=0; i<5; i++){
        ret = check_csm330(net);
        mdelay(10);
        if (!ret)
            break;
    }
    
    if (i == 5) {
        dev_err(&serdev->dev,"csm300 check id error\n");
        serdev_device_close(serdev);
        goto out_free;
    }
    else
        dev_info(&serdev->dev,"csm300 check id succeed\n");
    
    serdev_device_close(serdev);
    
    mutex_init(&priv->csm_lock);
    
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
    timer_setup(&priv->timer, send_data_start, 0);
#else
    init_timer(&(priv->timer));

    priv->timer.function = send_data_start;

    priv->timer.data = (unsigned long)net;
#endif
    
    SET_NETDEV_DEV(net, &serdev->dev);
    
    
    ret = register_candev(net);
    if (ret)
        goto out_free;

    netdev_info(net, "csm330 successfully initialized.\n");
    return 0;

out_free:
    free_candev(net);
    dev_err(&serdev->dev, "Probe failed\n");
    return ret;
}

static void csm330_uart_remove(struct serdev_device *serdev)
{
    struct csm330_priv *priv = serdev_device_get_drvdata(serdev);
    struct net_device *net = priv->net;
    cancel_work_sync(&priv->tx_work);
    unregister_candev(net);
    free_candev(net);
    serdev_device_close(serdev);

}


static struct serdev_device_driver csm330_uart_driver = {
    .probe = csm330_uart_probe,
    .remove = csm330_uart_remove,
    .driver = {
        .name = CSM330A_DEV_NAME,
        .of_match_table = of_match_ptr(csm330_uart_of_match),
    },
};

module_serdev_device_driver(csm330_uart_driver);

MODULE_DESCRIPTION("zhiyuan CSM330A UART driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(CSM330A_DEV_VERSION);
