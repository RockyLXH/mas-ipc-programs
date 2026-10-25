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
