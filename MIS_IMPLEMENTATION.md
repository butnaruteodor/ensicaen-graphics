# Multiple Importance Sampling Path Tracer Implementation

## Overview

This document describes the implementation of `path_mis.cpp`, a Monte Carlo path tracer that uses Multiple Importance Sampling (MIS) to reduce variance compared to single-strategy samplers.

## Key Concepts

### Balance Heuristic

The balance heuristic weights contributions from different sampling strategies:

$$w_a(p_a, p_b) = \frac{p_a}{p_a + p_b}$$

where $p_a$ and $p_b$ are probability densities of two sampling strategies. This ensures:
- Both probabilities are expressed in the same measure (solid angle in our case)
- Variance is reduced compared to either strategy alone
- The estimator remains unbiased

### Two Sampling Strategies

**Strategy 1: Light Sampling**
- Sample a position on a light source
- Evaluate BSDF at the resulting direction
- Weight by: $w_{\text{light}} = \frac{p_{\text{light}}}{p_{\text{light}} + p_{\text{BSDF}}}$

**Strategy 2: BSDF Sampling**
- Sample a direction from the BSDF
- If it hits a light, use it to estimate direct illumination
- Weight by: $w_{\text{BSDF}} = \frac{p_{\text{BSDF}}}{p_{\text{light}} + p_{\text{BSDF}}}$

## Implementation Details

### Light Sampling Path (Lines 72-102)

```cpp
// Sample one light uniformly
const Emitter *emitter = emitters[lightIdx];
EmitterQueryRecord lRec;
lRec.ref = its.p;

float pdfLight;
Color3f Le = emitter->sample(lRec, sampler->next2D(), pdfLight);
```

Key steps:
1. **Sample light**: `emitter->sample()` returns position, normal, radiance, and PDF (per-area)
2. **Convert PDF**: Convert from per-area to solid angle:
   $$p_{\Omega}^{\text{light}} = p_{\text{area}} \cdot \frac{r^2}{\cos\theta_{\text{light}}}$$
3. **Compute BSDF PDF**: Evaluate `its.bsdf->pdf()` for the sampled direction
4. **MIS Weight**: Apply balance heuristic
5. **Account for uniform light selection**: Multiply by $N$ (number of lights)

### BSDF Sampling Path (Lines 112-157)

```cpp
BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
Color3f bsdfSample = its.bsdf->sample(bRec, sampler->next2D());
float pdfBsdf = its.bsdf->pdf(bRec);
```

Key steps:
1. **Sample BSDF**: Get sampled direction and implicit weight (includes cosine)
2. **Trace ray**: Follow the sampled direction
3. **Check for light hit**: If ray intersects an emitter:
   - Evaluate emitter at intersection point
   - Query light's PDF at this point: `bsdfIts.emitter->pdf(pdfRec)`
   - Convert light PDF to solid angle
   - Apply MIS weight: $w_{\text{BSDF}} = \frac{p_{\text{BSDF}}}{p_{\text{light}} + p_{\text{BSDF}}}$
4. **Accumulate**: Add weighted contribution to path

## PDF Conversions

### Per-Area to Solid Angle

When a light samples a position with PDF per unit area:
$$p_{\Omega} = p_A \cdot \frac{r^2}{\cos\theta_{\text{light}}}$$

where:
- $r$ = distance between shading point and light
- $\cos\theta_{\text{light}}$ = cosine at the light surface

### Handling Multiple Lights

When uniformly selecting 1 light from $N$:
$$p_{\text{total}} = \frac{1}{N} \cdot p_{\text{light}}$$

In MIS weighting, this affects both the numerator and denominator:
$$w = \frac{p_{\text{light}} / N}{p_{\text{light}} / N + p_{\text{BSDF}}} = \frac{p_{\text{light}}}{p_{\text{light}} + N \cdot p_{\text{BSDF}}}$$

## Geometry Term

Both strategies use the complete geometry term:
$$G = \frac{\cos\theta_{\text{shading}} \cdot \cos\theta_{\text{light}}}{r^2}$$

This accounts for:
- Foreshortening at the shading point
- Foreshortening at the light
- Inverse-square falloff with distance

## Variance Reduction

MIS provides variance reduction compared to:
- **Light sampling alone**: Fails when BSDF is sharp (e.g., glossy)
- **BSDF sampling alone**: Fails when light is small (hard to hit)
- **Path tracing**: Converges faster with combined strategies

The balance heuristic automatically weights strategies based on their relative probabilities, giving preference to more probable sampling strategies.

## Files

- `src/path_mis.cpp` - Main MIS integrator implementation
- `scenes/pa5/cbox/cbox_path_mis.xml` - Test scene for Cornell box with MIS
- `CMakeLists.txt` - Updated to include path_mis.cpp

## Rendering

```bash
cd /home/teo/repos/ensicaen-graphics
make -j4
./nori scenes/pa5/cbox/cbox_path_mis.xml -o cbox_mis.exr
```

Expected improvements over `path_mats` (single-strategy):
- Faster convergence (lower variance)
- Better handling of mixed materials
- Smoother rendered result with equal sample count

## References

The balance heuristic is discussed in:
- Veach & Guibas, "Optimally Combining Sampling Techniques for Monte Carlo Rendering" (SIGGRAPH 1995)
- Eric Veach, PhD Thesis "Robust Monte Carlo Methods for Light Transport Simulation" (1997)
