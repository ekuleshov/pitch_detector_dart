import 'package:pitch_detector_dart/algorithm/pitch_algorithm.dart';
import 'package:pitch_detector_dart/pitch_detector_result.dart';

//  An implementation of the AUBIO_YIN pitch tracking algorithm.
//  This is a port of the TarsosDSP library developed by Joren Six and Paul Brossier at IPEM, University Ghent
//  Original implementation : https://github.com/JorenSix/TarsosDSP
//
//  Ported by Techpotatoes - Lucas Bento

class Yin extends PitchAlgorithm {
  /// The default YIN threshold value. Should be around 0.10~0.15. See YIN paper for more information.
  static final double DEFAULT_THRESHOLD = 0.20;

  // The default size of an audio buffer (in samples).
  // static final int DEFAULT_BUFFER_SIZE = 2048;

  // The default overlap of two consecutive audio buffers (in samples).
  // static final int DEFAULT_OVERLAP = 1536;

  final double _threshold;
  final double _sampleRate;
  final List<double> _yinBuffer;

  Yin(double audioSampleRate, int bufferSize, [double? threshold])
      : this._sampleRate = audioSampleRate,
        this._threshold = threshold ?? DEFAULT_THRESHOLD,
        this._yinBuffer = List<double>.filled(bufferSize ~/ 2, 0.0);

  @override
  PitchDetectorResult getPitch(final List<double> audioBuffer) {

    // step 2
    _difference(audioBuffer);

    // step 3
    _cumulativeMeanNormalizedDifference();

    PitchDetectorResult result = PitchDetectorResult.empty();

    // step 4
    final int tauEstimate = _absoluteThreshold(result);

    // step 5
    if (tauEstimate != -1) {
      final double betterTau = _parabolicInterpolation(tauEstimate);

      // step 6
      // TODO Implement optimization for the AUBIO_YIN algorithm.
      // 0.77% => 0.5% error rate,
      // using the data of the YIN paper
      // bestLocalEstimate()

      // conversion to Hz
      result.pitch = _sampleRate / betterTau;
    } else {
      // no pitch found
      result.pitch = -1;
    }

    return result;
  }

  //Implements the difference function as described in step 2 of the YIN
  void _difference(final List<double> audioBuffer) {
    for (int tau = 0; tau < _yinBuffer.length; tau++) {
      _yinBuffer[tau] = 0;
    }
    for (int tau = 1; tau < _yinBuffer.length; tau++) {
      for (int index = 0; index < _yinBuffer.length && index + tau < audioBuffer.length; index++) {
        double delta = audioBuffer[index] - audioBuffer[index + tau];
        _yinBuffer[tau] += delta * delta;
      }
    }
  }

  /// The cumulative mean normalized difference function as described in step 3 of the YIN paper.
  void _cumulativeMeanNormalizedDifference() {
    _yinBuffer[0] = 1;
    double runningSum = 0;
    for (int tau = 1; tau < _yinBuffer.length; tau++) {
      runningSum += _yinBuffer[tau];
      _yinBuffer[tau] *= tau / runningSum;
    }
  }

  /// Implements step 4 of the AUBIO_YIN paper.
  int _absoluteThreshold(PitchDetectorResult result) {
    // Uses another loop construct
    // than the AUBIO implementation
    int tau;
    // first two positions in yinBuffer are always 1
    // So start at the third (index 2)
    for (tau = 2; tau < _yinBuffer.length; tau++) {
      if (_yinBuffer[tau] < _threshold) {
        while (tau + 1 < _yinBuffer.length &&
            _yinBuffer[tau + 1] < _yinBuffer[tau]) {
          tau++;
        }
        // found tau, exit loop and return
        // store the probability
        // From the YIN paper: The threshold determines the list of
        // candidates admitted to the set, and can be interpreted as the
        // proportion of aperiodic power tolerated
        // within a periodic signal.
        //
        // Since we want the periodicity and and not aperiodicity:
        // periodicity = 1 - aperiodicity
        result.probability = 1 - _yinBuffer[tau];
        break;
      }
    }

    // if no pitch found, tau => -1
    if (tau == _yinBuffer.length || _yinBuffer[tau] >= _threshold) {
      tau = -1;
      result.probability = 0;
      result.pitched = false;
    } else {
      result.pitched = true;
    }

    return tau;
  }

  /// Implements step 5 of the AUBIO_YIN paper. It refines the estimated tau
  /// value using parabolic interpolation. This is needed to detect higher
  /// frequencies more precisely. See http://fizyka.umk.pl/nrbook/c10-2.pdf and
  ///  for more background
  /// http://fedc.wiwi.hu-berlin.de/xplore/tutorials/xegbohtmlnode62.html
  double _parabolicInterpolation(final int tauEstimate) {
    final int x0 = tauEstimate < 1 ? tauEstimate : tauEstimate - 1;
    final int x2 = tauEstimate + 1 < _yinBuffer.length ? tauEstimate + 1 : tauEstimate;

    final double betterTau;
    if (x0 == tauEstimate) {
      betterTau = (_yinBuffer[tauEstimate] <= _yinBuffer[x2] ? tauEstimate : x2).toDouble();
    } else if (x2 == tauEstimate) {
      betterTau = (_yinBuffer[tauEstimate] <= _yinBuffer[x0] ? tauEstimate : x0).toDouble();
    } else {
      double s0 = _yinBuffer[x0];
      double s1 = _yinBuffer[tauEstimate];
      double s2 = _yinBuffer[x2];
      // fixed AUBIO implementation, thanks to Karl Helgason:
      // (2.0f * s1 - s2 - s0) was incorrectly multiplied with -1
      betterTau = tauEstimate + (s2 - s0) / (2 * (2 * s1 - s2 - s0));
    }
    return betterTau;
  }
}
