

// Includes the matrix that first converts from linear RGB to XYZ.
const mat3 M1 = mat3(
  0.4122214708, 0.2119034982, 0.0883024619, // column 0
  0.5363325363, 0.6806995451, 0.2817188376, // column 1
  0.0514459929, 0.1073969566, 0.6299787005 // column 2
);
const mat3 M2 = mat3(
   0.2104542553,  1.9779984951,  0.0259040371,
   0.7936177850, -2.4285922050,  0.7827717662,
  -0.0040720468,  0.4505937099, -0.8086757660
);

const mat3 M2inv = mat3(
  1.0,           1.0,           1.0,
  0.3963377774, -0.1055613458, -0.0894841775,
  0.2158037573, -0.0638541728, -1.2914855480
);
const mat3 M1inv = mat3(
   4.0767416621, -1.2684380046, -0.0041960863,
  -3.3077115913,  2.6097574011, -0.7034186147,
   0.2309699292, -0.3413193965,  1.7076147010
);

vec3 toOklab(vec3 linearColor) {
  vec3 lms = M1  * linearColor;
  lms = pow(lms, vec3(1./3));
  return M2 * lms;
}

vec3 fromOklab(vec3 oklabColor) {
  vec3 lms = M2inv * oklabColor;
  lms = pow(lms, vec3(3));
  return M1inv * lms;
}

vec3 oklabMix(vec3 linearA, vec3 linearB, float t) {
  vec3 oklabA = toOklab(linearA);
  vec3 oklabB = toOklab(linearB);
  vec3 oklabMixed = mix(oklabA, oklabB, t);
  return fromOklab(oklabMixed);
}
