#ifndef RECEIVER_H
#define RECEIVER_H

class Receiver {
	public:
		// Struct to store the receiver values. All values are in the range [-1,1]
	    struct values{
			double roll;
			double pitch;
			double yaw;
			double throttle;
			double aux[2];
		};
		Receiver(); 
		values getData();
		void print();

};

#endif
