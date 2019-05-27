#ifdef CONFIG_SUSPEND

#include <linux/suspend.h>

int get_RTK_PM_STATE(void) {
	return RTK_PM_STATE;
}

EXPORT_SYMBOL(get_RTK_PM_STATE);

#endif //CONFIG_SUSPEND

int initial_flag2=0;

int get_RTK_initial_flag(void) {
        return initial_flag2;
}

EXPORT_SYMBOL(get_RTK_initial_flag);

void set_RTK_initial_flag(int flag) {
        initial_flag2=flag;
}

EXPORT_SYMBOL(set_RTK_initial_flag);

