# filter Data Filtering

## Overview

This directory contains data-filtering utilities for removing noise from sensor data.

## File List

| File | Description |
|------|----------|
| `MeanFilter.java` | Moving average filter |
| `AngleMeanFilter.java` | Angle-specific moving average filter (avoids angle wrap-around jumps) |
| `AngleWeightedMeanFilter.java` | Weighted moving average angle filter |
| `EMA.java` | Exponential moving average filter |

## Filter Selection Guide

| Scenario | Recommended Filter | Reason |
|------|-----------|------|
| Ordinary sensor data filtering | `MeanFilter` | Simple and efficient, suitable for most scenarios |
| Angle data filtering | `AngleMeanFilter` | Handles angle periodicity, avoids jumps at +/-180 degrees |
| Weighted angle filtering | `AngleWeightedMeanFilter` | Supports weighting by confidence |
| Low-latency, smooth output needed | `EMA` | Single-value recursion, small memory footprint with tunable response |

## MeanFilter Usage

### Features

- Sliding-window moving average
- Automatically handles NaN and infinite values (invalid values are discarded and the current mean is returned)
- Provides statistics such as variance and sample count

### Example

```java
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

// Create a filter with window size 10
MeanFilter filter = new MeanFilter(10);

// Feed samples step by step and get the filtered value
double filteredValue = filter.filter(rawSensorValue);

// Get the current mean
double mean = filter.getMean();

// Get the variance
double variance = filter.getVariance();

// Received sample count and window size
int count = filter.getCount();
int window = filter.getWindowSize();

// Reset the filter
filter.reset();
```

### Method Signatures

| Method | Description |
|------|------|
| `MeanFilter(windowSize)` | Constructor, windowSize must be greater than 0 |
| `filter(newValue)` | Add a new sample and return the filtered value |
| `getMean()` | Get the current mean |
| `getVariance()` | Get the current variance |
| `getCount()` | Get the received sample count |
| `getWindowSize()` | Get the window size |
| `reset()` | Reset the filter |

## AngleMeanFilter Usage

### Features

- Uses vector (circular-mean) averaging to avoid angle wrap-around jumps
- Supports both radian and degree inputs
- Provides a consistency metric

### Example

```java
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;

// Create an angle filter with window size 5
AngleMeanFilter angleFilter = new AngleMeanFilter(5);

// Feed radians
double filteredRad = angleFilter.filter(angleRad);

// Feed degrees
double filteredDeg = angleFilter.filterDegrees(angleDeg);

// Get the current mean angle
double avgAngle = angleFilter.getAverageAngle();

// Get the consistency metric (0-1, closer to 1 means more consistent)
double consistency = angleFilter.getConsistency();

// Reset the filter
angleFilter.reset();
```

### Method Signatures

| Method | Description |
|------|------|
| `AngleMeanFilter(windowSize)` | Constructor, windowSize must be greater than 0 |
| `filter(angleRad)` | Add a radian value and return the mean angle |
| `filterDegrees(angleDeg)` | Add a degree value and return the mean angle (in degrees) |
| `getAverageAngle()` | Get the current mean angle |
| `getConsistency()` | Get the consistency metric |
| `reset()` | Reset the filter |

## AngleWeightedMeanFilter Usage

### Features

- Supports weighted angle averaging
- Weights can be adjusted dynamically from sensor confidence

### Example

```java
import org.firstinspires.ftc.teamcode.utility.filter.AngleWeightedMeanFilter;

AngleWeightedMeanFilter filter = new AngleWeightedMeanFilter(5);

// Weighted filtering (weight acts as the vector magnitude in the average)
double filtered = filter.filter(angleRad, confidence);

// Weighted degree filtering
double filteredDeg = filter.filterDegrees(angleDeg, confidence);

// Get the current mean angle and consistency metric
double avgAngle = filter.getAverageAngle();
double consistency = filter.getConsistency();

// Reset the filter
filter.reset();
```

### Method Signatures

| Method | Description |
|------|------|
| `AngleWeightedMeanFilter(windowSize)` | Constructor, windowSize must be greater than 0 |
| `filter(angleRad, weight)` | Add a weighted radian value and return the mean angle |
| `filterDegrees(angleDeg, weight)` | Add a weighted degree value and return the mean angle (in degrees) |
| `getAverageAngle()` | Get the current mean angle |
| `getConsistency()` | Get the consistency metric |
| `reset()` | Reset the filter |

## EMA Usage

### Features

- Exponential moving average with formula `filteredValue = alpha * x + (1 - alpha) * filteredValue`
- An `alpha` closer to 1 means weaker smoothing and faster response; closer to 0 means stronger smoothing and slower response
- Supports creation from an equivalent window size (`alpha = 2 / (n + 1)`)

### Example

```java
import org.firstinspires.ftc.teamcode.utility.filter.EMA;

// Specify the smoothing factor directly
EMA ema = new EMA(0.2);

// Or create from an equivalent window size
EMA ema2 = EMA.fromWindow(10);

// Update step by step and get the filtered value (the first call returns the input as the initial value)
double filtered = ema.update(rawSensorValue);

// Adjust the smoothing factor at runtime
ema.setAlpha(0.1);

// Get the current state
double value = ema.getFilteredValue();
double alpha = ema.getAlpha();
boolean hasInit = ema.hasInitialValue();

// Reset the filter
ema.reset();
```

### Method Signatures

| Method | Description |
|------|------|
| `EMA(alpha)` | Constructor, alpha range (0, 1] |
| `fromWindow(n)` | Create an instance from equivalent window size n |
| `update(x)` | Feed a raw value and return the filtered value |
| `setAlpha(alpha)` | Set the smoothing factor |
| `getFilteredValue()` | Get the current filtered value |
| `getAlpha()` | Get the smoothing factor |
| `hasInitialValue()` | Whether a valid sample has been received |
| `reset()` | Reset the filter |

### Notes

1. All filters handle NaN/infinite values differently: `MeanFilter` and `EMA` ignore invalid values, while `AngleMeanFilter`/`AngleWeightedMeanFilter` return `NaN` when the vector sum is zero
2. `windowSize` must be greater than 0, otherwise the constructor throws `IllegalArgumentException`
3. `alpha` of `EMA` must be in the range (0, 1], otherwise it throws `IllegalArgumentException`