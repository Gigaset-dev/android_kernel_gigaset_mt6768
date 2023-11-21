/*
 * Privileged & confidential  All Rights/Copyright Reserved by FocalTech.
 *       ** Source code released bellows and hereby must be retained as
 * FocalTech's copyright and with the following disclaimer accepted by
 * Receiver.
 *
 * "THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL THE FOCALTECH'S AND ITS AFFILIATES'DIRECTORS AND OFFICERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE."
 */

#ifndef __FOCALTECH_CORE_H__
#define __FOCALTECH_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define HOST_MCU_DRIVER_VERSION                  	"FocalTech MCU V1.0 20220316"

#define FTS_INFO(fmt, ...)							printk("[FTS/I]%s:"fmt"\n", __func__, ##__VA_ARGS__)
#define FTS_ERROR(fmt, ...)							printk("[FTS/E]%s:"fmt"\n", __func__,  ##__VA_ARGS__)
#define FTS_DEBUG(fmt, ...)	    					printk("[FTS/D]%s:"fmt"\n", __func__, ##__VA_ARGS__)

#define INTERVAL_READ_REG                   		200  /* unit:ms */


#define FTS_CMD_READ_ID                     		0x90

/* chip id */
#define FTS_CHIP_IDH								0x64
#define FTS_CHIP_IDL								0x56

/* register address */
#define FTS_REG_CHIP_ID                     		0xA3
#define FTS_REG_CHIP_ID2                    		0x9F
#define FTS_REG_FW_VER                      		0xA6
#define FTS_REG_UPGRADE                         	0xFC

/*
 * Gesture function enable
 * default: disable
 */
#define FTS_GESTURE_EN                          	0

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
 * Structures of point information
 *
 * @x: X coordinate of this point
 * @y: Y coordinate of this point
 * @p: pressure value of this point
 * @type: event type of this point, 0 means down event,
 *		  1 means up event, 2 means contact event
 * @id: ID of this point
 * @area: touch area of this point
 */
struct fts_ts_event {
    int x;		/*x coordinate */
    int y;		/*y coordinate */
    int p;		/* pressure */
    int type;	/* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id; 	/*touch ID */
    int area;   /*touch area*/
};


struct fts_ts_data {
    int suspended;		   /* suspended state, 1: suspended mode, 0:not suspended mode */
    int esd_support;	   /* esd enable or disable, default: disable */
    int gesture_support;   /* gesture enable or disable, default: disable */
};


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct fts_ts_data *fts_data;

void fts_msleep(unsigned long msec);

/* communication interface */
int fts_read(uint8_t addr, uint8_t *data, uint16_t datalen);
int fts_read_reg(uint8_t addr, uint8_t *value);
int fts_write(uint8_t addr, uint8_t *data, uint16_t datalen);
int fts_write_reg(uint8_t addr, uint8_t value);

int fts_ts_init(void);
int fts_ts_suspend(void);
int fts_ts_resume(void);


#endif /* __FOCALTECH_CORE_H__ */
