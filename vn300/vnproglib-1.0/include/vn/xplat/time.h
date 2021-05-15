// VectorNav Programming Library v1.0.0.1
// Copyright (c) 2015 VectorNav Technologies, LLC
#ifndef _VNXPLAT_TIME_H_
#define _VNXPLAT_TIME_H_

namespace vn {
namespace xplat {

/// \brief Provides simple timing capabilities.
class Stopwatch
{

public:
	
	/// \brief Creates a new Stopwatch and starts timing.
	Stopwatch();

	~Stopwatch();

	/// \brief Resets the Stopwatch.
	void reset();

	/// \brief Gets the elapsed time in milliseconds.
	///
	/// \return The elapsed time in milliseconds.
	float elapsedMs();

private:
	struct Impl;
	Impl *_pi;
};

}
}

#endif
