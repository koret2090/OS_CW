#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
    int sim_fd;
    int x, y;
    char buffer[10];
    /* Open the sysfs coordinate node */
    sim_fd = open("/sys/devices/platform/vms/coordinates", O_RDWR);
    if (sim_fd < 0)
    {
        perror("Couldn't open vms coordinate file\n");
        exit(-1);
    }

    while (1) 
    {
        /* Generate random relative coordinates */
        x = random()%100;
        y = random()%100;
        if (x%2) x = -x; if (y%2) y = -y;
        /* Convey simulated coordinates to the virtual mouse driver */
        sprintf(buffer, "%d %d %d", x, y, 0);
        write(sim_fd, buffer, strlen(buffer));
        fsync(sim_fd);
        sleep(0.5);
    }
    close(sim_fd);
}

void write_coords(int x, int y)
{
    int sim_fd;
    char buffer[10];
    /* Open the sysfs coordinate node */
    sim_fd = open("/sys/devices/platform/vms/coordinates", O_RDWR);
    if (sim_fd < 0)
    {
        perror("Couldn't open vms coordinate file\n");
        exit(-1);
    }


    /* Generate random relative coordinates */
    x = x % 100;
    y = y % 100;
    
    /* Convey simulated coordinates to the virtual mouse driver */
    sprintf(buffer, "%d %d %d", x, y, 0);
    write(sim_fd, buffer, strlen(buffer));
        
    
    close(sim_fd);
}


static void write_coords(int x, int y)
{
    char buffer[64];
    /* Open the sysfs coordinate node */
	struct file *f = filp_open("/sys/devices/platform/vms/coordinates", O_RDWR, 0);
	if (!f || IS_ERR(f))
	{
		f = NULL;
		printk("Couldn't open vms coordinate file");
	}

	/* Generate random relative coordinates */
	x = x % 100;
	y = y % 100;
	
	/* Convey simulated coordinates to the virtual mouse driver */
	sprintf(buffer, "%d %d %d", x, y, 0);
	vfs_write(f, buffer, strlen(buffer), 0);
	//fsync(sim_fd);
    //filp_close(f, NULL);
}


extern int send_coordinates(char dx, char dy)
{
    printk(KERN_INFO "received send_mouse_coordinates %d %d\n", dx, dy);
    
    char buf[32];
	int len;
    
    sprintf(buf, "%d %d %d", dx, dy, 0);

    len = strlen(buf);

	memset(my_log, 0, MY_LOG_SIZE);
	my_log_len = 0;
	
    strcat(my_log, buf);
    my_log_len += len;
    
    return 0;
}