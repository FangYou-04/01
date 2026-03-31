#include "ExtendedKalman.hpp"
#include "Congfig.hpp"
#include <opencv2/opencv.hpp>

ExtendedKalman::ExtendedKalman()
    : m_lastTime(0),
      m_dt(0),
      m_initialized(false),
      m_predicetedYaw(0.0f)
{
                                                                    
}