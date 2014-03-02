/*
 */

struct isp158x_udc_mach_info {
        void (*udc_command)(int cmd);
#define	ISP158X_UDC_CMD_INIT		0       
#define	ISP158X_UDC_CMD_CONNECT		1	/* let host see us */
#define	ISP158X_UDC_CMD_DISCONNECT	2	/* so host won't see us */
};

