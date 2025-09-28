// Returns multiplier for light, 0 for total shadow, 1 for total light.
float shadowPct() {
  vec3 shadowNdc = shadowPos.xyz / shadowPos.w;
  vec2 shadowUv = shadowNdc.xy * 0.5 + vec2(0.5);

  if (any(lessThan(shadowUv, vec2(0))) ||
      any(greaterThan(shadowUv, vec2(1)))) {
    return 1;
  }

  float shadowZ = texture(sampler2D(shadowTex, shadowSampler), shadowUv).r;
  // Clipped by projection, no shadow.
  if (shadowZ == 1.0 || shadowZ == 0.0) {
    return 1;
  }

  vec3 lightDir = -normalize(vec3(global.view * global.lights[0].vec));
  vec3 vNorm = normalize(fragNormal);
  float bias = max(0.01 * (1.0 - dot(vNorm, lightDir)), 0.00005);

  if (shadowNdc.z + bias < shadowZ) {
    return 0;
  } else {
    return 1;
  }
}
