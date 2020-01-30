
#ifndef INCLUDE_WATCHDOG_H
#define INCLUDE_WATCHDOG_H

#include <thread>
#include <ros/duration.h>


class WatchDog {
public:
	WatchDog(ros::Duration timeout_)
		:	timeout(timeout_)
	{
	}

	~WatchDog()
	{
		stop();
	}

	void start()
	{
		stop();
		do_run = true;
		thread = std::thread(&WatchDog::run_loop, this);
	}

	void stop()
	{
		do_run = false;
		if(thread.joinable()) {
			thread.join();
		}
	}

	void tickle()
	{
		is_ok = true;
	}

protected:
	virtual void handle_timeout() = 0;		// callback to be implemented by user

private:
	void run_loop()
	{
		while(do_run)
		{
			is_ok = false;
			timeout.sleep();

			if(!is_ok) {
				handle_timeout();
			}
		}
	}

private:
	volatile bool do_run = false;
	volatile bool is_ok = false;
	ros::Duration timeout;
	std::thread thread;

};


#endif // INCLUDE_WATCHDOG_H
