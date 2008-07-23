#ifndef PID_HPP
#define PID_HPP

class Pid
{
	public:
		Pid(float p, float i, float d, unsigned int windup);
		~Pid();

		unsigned int windup() const { return _windup; }

		/** Runs the pid loop and returns the error */
		float run(float err);

		/** clear any windup */
		void clearWindup();

		float kp, ki, kd;

	private:
		Pid(Pid&);
		Pid& operator &= (Pid&);

		/** amount to sum up */
		unsigned int _windup;

		unsigned int _windupLoc;

		float _errSum;

		float _lastErr;

		float* _oldErr;
};

#endif /* PID_HPP */
