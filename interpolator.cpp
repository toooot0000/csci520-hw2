#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <utility>
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

/**
 *
 * @param p0
 * @param p1
 * @return return [p
 */
template<class T>
static std::pair<T, T> GetControlPoint(const T& p0, const T& p1, const T& p2, T (*lerp)(const T&, const T&, double)){
    auto anBar = lerp(lerp(p0, p1, 2.0), p2, 0.5);
    return {lerp(p1, anBar, 1.0/3), lerp(p1, anBar, -1.0/3)};
}

template <class T>
static const T& min (const T& a, const T& b) {
    return b >= a ? a : b;     // or: return !comp(b,a)?a:b; for version (2)
}

template <class T>
static T Lerp(const T& start, const T& end, double t){
    return (1 - (t)) * (start) + (t) * (end);
}


void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int keyframe[3] = {0,N + 1,min((N + 1) * 2, inputLength - 1)};

    Posture *p0 = pInputMotion->GetPosture(keyframe[0]),
            *p1 = pInputMotion->GetPosture(keyframe[1]),
            *p2 = pInputMotion->GetPosture(keyframe[2]);

    vector posAn0 = Lerp(p0->root_pos, Lerp(p2->root_pos, p1->root_pos, 2.0), 1.0/3),
        posAn1, posBn1;

    vector boneAn0[MAX_BONES_IN_ASF_FILE], boneAn1[MAX_BONES_IN_ASF_FILE], boneBn1[MAX_BONES_IN_ASF_FILE];
    for(int i = 0; i<MAX_BONES_IN_ASF_FILE; i++){
        boneAn0[i] = Lerp(p0->bone_rotation[i], Lerp(p2->bone_rotation[i], p1->bone_rotation[i], 2.0), 1.0/3);
    }

    while(keyframe[1] < inputLength){
        printf("Lerp from frame#%d to frame#%d\n", keyframe[0], keyframe[1]);

        p0 = pInputMotion->GetPosture(keyframe[0]);
        p1 = pInputMotion->GetPosture(keyframe[1]);
        pOutputMotion->SetRootPos(keyframe[0], p0->root_pos);
        pOutputMotion->SetRootPos(keyframe[1], p1->root_pos);
        for(int i = 0; i<MAX_BONES_IN_ASF_FILE; i++){
            pOutputMotion->SetBoneRotation(keyframe[0], i, p0->bone_rotation[i]);
            pOutputMotion->SetBoneRotation(keyframe[1], i, p1->bone_rotation[i]);
        }

        if(keyframe[2] < inputLength){
            p2 = pInputMotion->GetPosture(keyframe[2]);
            auto next = GetControlPoint(p0->root_pos, p1->root_pos, p2->root_pos, Lerp);
            posAn1 = next.first;
            posBn1 = next.second;

            for(int i = 0; i<MAX_BONES_IN_ASF_FILE; i++){
                auto boneNext = GetControlPoint(p0->bone_rotation[i], p1->bone_rotation[i], p2->bone_rotation[i], Lerp);
                boneAn1[i] = boneNext.first;
                boneBn1[i] = boneNext.second;
            }
        } else {
            auto pm1 = pInputMotion->GetPosture(keyframe[0] - N - 1);
            posBn1 = Lerp(p1->root_pos, Lerp(pm1->root_pos, p0->root_pos, 2.0), 1.0/3);
            for(int i = 0; i<MAX_BONES_IN_ASF_FILE; i++){
                boneBn1[i] = Lerp(p1->bone_rotation[i], Lerp(pm1->bone_rotation[i], p0->bone_rotation[i], 2.0), 1.0/3);
            }
        }

        for(int f = 1; f < N + 1; f++ ){
            Posture inter;
            auto t = (double)(f) / (N + 1);
            inter.root_pos = DeCasteljauEuler(t, p0->root_pos, posAn0, posBn1, p1->root_pos);

            for(int i = 0; i < MAX_BONES_IN_ASF_FILE; i++){
                inter.bone_rotation[i] = DeCasteljauEuler(t, p0->bone_rotation[i], boneAn0[i], boneBn1[i], p1->bone_rotation[i]);
            }
            pOutputMotion->SetPosture(f + keyframe[0], inter);
        }

        std::swap(posAn0, posAn1);
        for(int i = 0; i<MAX_BONES_IN_ASF_FILE; i++){
            std::swap(boneAn0[i], boneAn1[i]);
        }


        keyframe[0] = keyframe[1];
        keyframe[1] = keyframe[2];
        keyframe[2] += N + 1;
    }

    for(int f = keyframe[0] + 1; f < inputLength; f++){
        pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
    }
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
  result = 2 * (p.Dot(q)) * q - p;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
    vector p[4] = {p0, p1, p2, p3};
    for(int i = 0; i< 4; i++){
        for(int j = 3; j > 0; j--){
            p[j] = (1 - t) * p[j-1] + t * p[j];
        }
    }
    return p[3];
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
    Quaternion<double> p[4] = {p0, p1, p2, p3};
    for(int i = 0; i< 4; i++){
        for(int j = 3; j > 0; j--){
            p[j] = (1 - t) * p[j-1] + t * p[j];
        }
    }
    return p[3];
}

