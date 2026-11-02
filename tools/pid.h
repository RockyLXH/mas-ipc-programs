/*
 * pid.h
 *
 * Created on: Jul 25, 2022
 * Author: RockyLiu
 */

#pragma once

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class PIDController
{
public:
	explicit PIDController(double, double, double, double, double, double);
	~PIDController() = default;

	PIDController(const PIDController &) = delete;
	PIDController &
	operator=(const PIDController &) = delete;

	double
	operator()(double error);

	const double &
	kp() const
	{
		return kp_;
	}
	void set_kp(const double kp)
	{
		kp_ = kp;
	}

	const double &
	ki() const
	{
		return ki_;
	}
	void set_ki(const double ki)
	{
		ki_ = ki;
	}

private:
	double kp_;
	double ki_;
	double kd_;
	double output_ramp_;
	double limit_;
	double ts_;

	double error_prev_;
	double output_prev_;
	double integral_prev_;
};

PIDController::PIDController(double kp, double ki, double kd = 0, double ramp = 1000,
							 double limit = 1, double ts = 0.00025)
	: kp_(kp), ki_(ki), kd_(kd), output_ramp_(ramp), limit_(limit), ts_(ts)
{
}

double PIDController::operator()(double error)
{
	double proportional = kp_ * error;

	double integral = integral_prev_ + ki_ * ts_ * 0.5f * (error + error_prev_);
	integral = CONSTRAIN(integral, -limit_, limit_);

	double derivative = kd_ * (error - error_prev_) / ts_;

	double output = proportional + integral + derivative;
	output = CONSTRAIN(output, -limit_, limit_);

	if (output_ramp_ > 0)
	{
		double output_rate = (output - output_prev_) / ts_;

		if (output_rate > output_ramp_)
			output = output_prev_ + output_ramp_ * ts_;
		else if (output_rate < -output_ramp_)
			output = output_prev_ - output_ramp_ * ts_;
	}

	integral_prev_ = integral;
	output_prev_ = output;
	error_prev_ = error;

	return output;
}