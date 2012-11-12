#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/parport.h>
#include <asm/uaccess.h>

#define LEAK_NAME	"leak"
#define LEAK_VERSION	"0.1"

#define LEAK_DEV	"leak"
#define LEAK_MAJOR	MISC_MAJOR
#define LEAK_MINOR	151

#define ON_COMMAND	"on"
#define OFF_COMMAND	"off"

static char pin_state = 0;
static int buffer_empty = 0;
static int leak_open_cnt = 0;
static int available_ports = 0;
static struct pardevice *parport_pins = 0;

MODULE_AUTHOR("Christian Lees");
MODULE_DESCRIPTION("Driver for the Iver leak detector");
MODULE_LICENSE("GPL");
/*
void
set_pins(void)
{
	if (parport_claim_or_block(parport_pins) < 0) {
		printk(KERN_ERR
		       "Could not claim the " LEAK_DEV " parallel port device\n");
		return;
	}
	parport_write_data(parport_pins->port, pin_state);
	parport_release(parport_pins);
}
*/

static void
leak_attach(struct parport *port)
{
	if (available_ports == 0) {
		parport_pins =
		    parport_register_device(port, LEAK_DEV, NULL, NULL,
					    NULL, 0, 0);

		if (parport_pins == 0)
			printk(KERN_ERR
			       "Could not associate " LEAK_DEV " device with parallel port #%d\n",
			       available_ports + 1);
		else
			printk(KERN_INFO
			       "Associated " LEAK_DEV " device with parallel port #%d\n",
			       available_ports + 1);
        
        if (parport_claim_or_block(parport_pins) < 0) {
		    printk(KERN_ERR
		       "Could not claim the " LEAK_DEV " parallel port device\n");
    		return -1;
	    }
        
        // set the direction to reverse, ie inputs           
        parport_data_reverse(parport_pins->port);
	}

	available_ports++;
}

static void
leak_detach(struct parport *port)
{
	if (available_ports == 1)
		parport_pins = 0;

	available_ports--;
}

static struct parport_driver leak_driver = {
	LEAK_NAME,
	leak_attach,
	leak_detach,
	{NULL}
};

static ssize_t
leak_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
	pin_state = parport_read_data(parport_pins->port);
    copy_to_user(buf, &pin_state, 1);

	return 1;
}

static int
leak_open(struct inode *inode, struct file *file)
{
	if (leak_open_cnt)
		return -EBUSY;
	else
		leak_open_cnt = 1;

	buffer_empty = 0;
	return 0;
}

static int
leak_release(struct inode *inode, struct file *file)
{
	leak_open_cnt = 0;
	return 0;
}

static struct file_operations leak_fops = {
	owner:THIS_MODULE,
	read:leak_read,
//	write:leak_write,
	open:leak_open,
	release:leak_release,
};

static struct miscdevice leak_dev = {
	LEAK_MINOR,
	LEAK_DEV,
	&leak_fops
};

int __init
leak_init(void)
{
	if (parport_register_driver(&leak_driver) != 0) {
		printk(KERN_ERR "Could not register the " LEAK_DEV " driver.\n");
		return -EIO;
	}

	if (misc_register(&leak_dev) != 0) {
		printk(KERN_ERR
		       "Could not register the misc device " LEAK_DEV " (%d, %d)\n",
		       LEAK_MAJOR, LEAK_MINOR);
		return -EIO;
	}

	printk(KERN_INFO "" LEAK_NAME " driver v%s loaded\n", LEAK_VERSION);

	//set_pins();
    
    
    
	return 0;
}

static void __exit
leak_cleanup(void)
{
	if (misc_deregister(&leak_dev) != 0)
		printk(KERN_ERR
		       "Cound not deregister the misc device " LEAK_DEV " (%d, %d)\n",
		       LEAK_MAJOR, LEAK_MINOR);

	parport_unregister_device(parport_pins);
	parport_unregister_driver(&leak_driver);

	printk(KERN_INFO "" LEAK_NAME " driver v%s unloaded\n", LEAK_VERSION);
}

module_init(leak_init);
module_exit(leak_cleanup);
