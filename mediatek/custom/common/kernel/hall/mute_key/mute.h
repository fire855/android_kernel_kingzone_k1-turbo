//#define MUTE_DEBUG_CODE

#define MUTE_DEVICE "MUTE"
#ifdef MUTE_DEBUG_CODE
#undef MUTE_DEBUG
#define MUTE_DEBUG(a,arg...) printk(MUTE_DEVICE ": " a, ##arg)
#define MUTE_FUNC()	printk(MUTE_DEVICE ": %s line=%d\n", __func__, __LINE__)
#else
#define MUTE_DEBUG(arg...)
#define MUTE_FUNC()
#endif

enum mute_report_state
{
	MUTE_YES =0,
	MUTE_NO = 1,
};