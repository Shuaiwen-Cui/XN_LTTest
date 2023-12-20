#include <xnode.h>
#include <app.h>

#if defined(INDICATOR)
int app_indicator(void);

int app_main(void)
{
	return app_indicator();
}

#elif defined(GATEWAY)
int app_gateway(void);

int app_main(void)
{
	return app_gateway();
}
#else
int app_sensor(void);

int app_main(void)
{
	return app_sensor();
}
#endif
