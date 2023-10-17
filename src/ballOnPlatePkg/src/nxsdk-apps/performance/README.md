## NxSDK Performance Test Suite

NxSDK comes with a performance test suite and associated Python decorators and metrics to create new benchmarks.

Performance tests are packaged within nxsdk-apps.

To run the full performance suite:

1. `cd nxsdk-apps`
2. `python -m performance.benchmark`

This creates a **benchmarks.html** file which can be opened in any web browser

---

#### Run subset of performance tests given a pattern

To run a **subset** of performance tests given a **pattern**, run:

`python -m performance.benchmark -p *basic*py`

This runs all tests which match the pattern **\*basic\*py** like test_basic_spike_injection.py

---

#### Run a single performance test

Each performance test comprises of a **main** function and can be run individually as:

`python -m performance.snips.test_basic_spike_injection`

You may run it similarly using an IDE for debugging or adding enhancements. The metrics will be printed onto the console along with the output of the test.

---

#### Measuring performance of user scripts

.. note:: Users can collect execution and energy statistics on their networks by leveraging the environment variables used by the performance suite to collect the metrics. See below relevant environment variables.

* _NXSDK_ENABLE_STATS=1_ : Set this to enable collection of all
metrics (Setting it inserts Execution and Energy Probes implicitly
into your network).
    * Usage: ``NXSDK_ENABLE_STATS=1 python <my_script.py>``

* _NXSDK_DISABLE_ENERGY_STATS=1_ : Energy statistics sometimes need
larger networks/longer runs and more sophisticated tuning. So it might
not be relevant for all scenarios. For instance, most of the
workloads in performance suite have it disabled. Note, by default,
NXSDK_ENABLE_STATS=1 inserts both kinds of performance probes.
    * Usage: ``NXSDK_DISABLE_ENERGY_STATS=1 NXSDK_ENABLE_STATS=1 python <my_script.py>``

Setting these environment variables will enable the performance logging and spit out all the metrics for all phases of run including compilation, execution and post-processing with energy measurements.

.. warning:: These metrics should be used only for **debugging** and **optimization** purposes. Competitive benchmarking should avoid these global environment variables which give you only a macro view of the execution and might not be fully accurate. Instead, use the underlying execution and energy probes with correct initialization parameters for accurate measurements. See Performance Profiling jupyter tutorial for examples.

---

#### Tag a performance measurment for reference/history

To run a benchmark and **tag** your run, execute:

`python -m performance.benchmark -t run1`

If you want to just run a subset and tag the same: `python -m performance.benchmark -t run-basic -p *basic*py`

_Tags are strings and only stand for your reference so that you may compare historical runs between software and hardware improvements/upgrades._

Each run is saved as a JSON file under a directory _.nxsdk-benchmarks_ with the name `tag`.json

---

#### Compare performance runs (Historial Comparisons)

To run an analysis of all your runs, we provide a benchmark comparison tool:

`python -m performance.benchmark_comparison`

This creates a **benchmarks_history.html** file which can be opened in any web browser.

This runs against the _.nxsdk-benchmarks_ directory and creates a comparison plot of all benchmarks which have run and tagged.

You might also point to a custom location. See `python -m performance.benchmark_comparison --help` for more options.

---

#### Add a new performance test

1. Create a directory structure (to group your tests) or add a file similar to existing benchmarks
2. Inherit your class from PerfLoggingHandler
3. Use the timeit decorator to add timing logs
    1. @timeit can be imported as: `from nxsdk.logutils.benchmark_utils import timeit`
    2. Then it can be added to any method as a decorator
4. Add a main method to make your test executable

---

#### Add a custom metric

1. Create a class inheriting from AbstractMetric. For e.g

    ```
    class MyCustomMetric(AbstractMetric, LowerIsBetterMetric):
    def __init__(self, value):
        AbstractMetric.__init__(self, "My Metric Label", value, "MyMetricUnit")
    ```

    Use appropriate labels and units for your metric.

    We provide two abstractions: **LowerIsBetterMetric** and **HigherIsBetterMetric**.
    Usually time and power related metrics might inherit from LowerIsBetterMetric while model accuracy might be a HigherIsBetterMetric.

2. Define your performance test by following the **Add a new performance test** mentioned above

3. To add your new custom metric, you need to import the @reportit decorator
    1. @reportit can be imported as: `from nxsdk.logutils.benchmark_utils import reportit, timeit`
    2. Then it can be added to any method as a decorator
    3. Ensure to return this metric from the function being benchmarked. For e.g., if you are testing method_x

    ```
    @timeit
    @reportit
    def time_method_x(self):
        ...some computation to test...
        return MyCustomMetric(some_value)
    ```

4. Run the performance test
