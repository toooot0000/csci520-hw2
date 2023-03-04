#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"

const double AngToRad = M_PI / 180.0;
const double RadToAng = 180.0 / M_PI;

static void ClampAngles(double angles[3]){
    for(int i=0; i<3; i++) {
        while(angles[i] < -180) angles[i] += 360;
        while(angles[i] > 180) angles[i] -= 360;
    }
}

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++) {
      angles[i] *= 180 / M_PI;
  }
}

void Interpolator::Euler2Rotation(const double angles[3], double R[9])
{
    double alpha = angles[0] * AngToRad, beta = angles[1] * AngToRad, gamma = angles[2] * AngToRad;
    double ca = cos(alpha), sa = sin(alpha),
        cb = cos(beta), sb = sin(beta),
        cg = cos(gamma), sg = sin(gamma);
    R[0] = cb * cg;
    R[1] = sa * sb * cg - ca * sg;
    R[2] = ca * sb * cg + sa * sg;
    R[3] = cb * sg;
    R[4] = sa * sb * sg + ca * cg;
    R[5] = ca * sb * sg - sa * cg;
    R[6] = - sb;
    R[7] = sa * cb;
    R[8] = ca * cb;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

static void FixNan(double angles[3]){
    for(int i = 0; i<3; i++){
        if(isnan(angles[i])){
            angles[i] = 0;
        }
    }
}



void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength) {
        int endKeyframe = startKeyframe + N + 1;

        Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++) {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                Quaternion<double> start, end, temp;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, start);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, end);
                temp = Slerp(t, start, end);
                Quaternion2Euler(temp, interpolatedPosture.bone_rotation[bone].p);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }
    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
    // students should implement this
    double m[9];
    ClampAngles(angles);
    Euler2Rotation(angles, m);
    q = Quaternion<double>::Matrix2Quaternion(m);
    q.Normalize();
}

void Interpolator::Quaternion2Euler(const Quaternion<double> & q, double angles[3])
{
  // students should implement this
    double m[9];
    q.Quaternion2Matrix(m);
    Rotation2Euler(m, angles);
    FixNan(angles);
    ClampAngles(angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{
#define clamp(x, lo, hi) fmin(fmax((x), (lo)), (hi))
  // students should implement this
    t = clamp(t, 0, 1);
    double cosTheta = qStart.Dot(qEnd);
    double sign = 1;
    if(cosTheta < 0) {
        cosTheta = -cosTheta;
        sign = -1;
    }
    double radTheta = acos(cosTheta);
    double sinTheta = sqrt(1 - cosTheta * cosTheta);
    Quaternion<double> result = (sign * sin((1 - t) * radTheta) / sinTheta) * qStart + sin(t*radTheta) / sinTheta * qEnd;
    return result;
#undef clamp
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

