
enum cart { X,Y,Z };

void computeIMU () 
{
  uint8_t axis;
  mpu6050_getADC();
  getEstimatedAttitude();
  annexCode();
  for(axis=0;axis<3;axis++)
    gyroData[axis] = gyroADC[axis];
}


//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

//#define GYRO_SCALE ((1998 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //(ITG3200 and MPU6050)
#define GYRO_SCALE ((2020 * PI)/(8191.75f * 180000000.0f)) //(MPU6050)
// +-2000/sec deg scale

typedef struct fp_vector {		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|

static t_fp_vector EstG = { 0.0, 0.0, (float)ACCRESO };
static t_int32_t_vector EstG32;
static t_fp_vector EstN = { 1000.0, 0.0, 0.0 };
static t_int32_t_vector EstN32;

static float scale = SCALECORR * GYRO_SCALE; // 0.25 * PIDCALCTIME * GYRO_SCALE

void getEstimatedAttitude()
{
  uint8_t axis;
  int32_t accMag = 0;
  float deltaGyroAngle[3];

  // Initialization
  for (axis = 0; axis < 3; axis++) 
  {
    deltaGyroAngle[axis] = gyroADC[axis] * scale;

    accLPF32[axis]    -= accLPF32[axis]>>ACC_LPF_FACTOR;
    accLPF32[axis]    += accADC[axis];
    accSmooth[axis]    = accLPF32[axis]>>ACC_LPF_FACTOR;

    accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
  }
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);

  rotateV(&EstG.V,deltaGyroAngle);
  rotateV(&EstN.V,deltaGyroAngle);
  
  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) 
  {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if (  72 < accMag && accMag < 133 )
    for (axis = 0; axis < 3; axis++) {
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }
  for (axis = 0; axis < 3; axis++)
    EstN32.A[axis] = EstN.A[axis];
    // a sanity check for the N vector length would be good.

  for (axis = 0; axis < 3; axis++)
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	

  // Attitude of the estimated vector
  int32_t sqGZ = sq(EstG32.V.Z);
  int32_t sqGX = sq(EstG32.V.X);
  int32_t sqGY = sq(EstG32.V.Y);
  int32_t sqGX_sqGZ = sqGX + sqGZ;
  float invmagXZ  = InvSqrt(sqGX_sqGZ);
  invG = InvSqrt(sqGX_sqGZ + sqGY);
  angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  angle[PITCH] = _atan2(EstG32.V.Y , invmagXZ*sqGX_sqGZ);

  act_heading = _atan2(
    EstN32.V.Z * EstG32.V.X - EstN32.V.X * EstG32.V.Z,
    EstN32.V.Y * invG * sqGX_sqGZ  - (EstN32.V.X * EstG32.V.X + EstN32.V.Z * EstG32.V.Z) * invG * EstG32.V.Y ); 
  act_heading = act_heading /10;
}

static float vel = 0.0f;
static int16_t accZoffset = 0; 
static int16_t accZsum = 0; 

void getEstAltChange()
{
  #if defined(ACCVARIO)
    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    int16_t accZ;
    accZ = (accSmooth[ROLL] * EstG32.V.X + accSmooth[PITCH] * EstG32.V.Y + accSmooth[YAW] * EstG32.V.Z) * invG;

    if (!f.ARMED) // guess that is the offset cancelling filter ?
    {
      accZoffset -= accZoffset>>3; // -= accZoffset / 8
      accZoffset += accZ;
      accZsum = 0;
    }
    accZ -= (accZoffset>>3);

    // Filter
    accZsum -= accZsum>>3;
    accZsum += accZ;
        
    vel = accZsum >> 3;

  #endif
}

