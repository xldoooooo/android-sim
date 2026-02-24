package com.xldoooooo.androidsim.sim

import com.xldoooooo.androidsim.logger.CsvLogger
import kotlinx.coroutines.*

class SimLoop(
    private val model:      DynamicsModel,
    private val controller: (t: Double, x: DoubleArray) -> DoubleArray,
    private val x0:         DoubleArray,
    private val tEnd:       Double  = 30.0,
    private val dt:         Double  = 0.005,
    private val useRK45:    Boolean = false,
    private val logger:     CsvLogger,
    private val onLogLine:  ((String) -> Unit)? = null
) {
    private var job: Job? = null

    fun start(scope: CoroutineScope) {
        job = scope.launch(Dispatchers.Default) {
            val rk4  = RungeKutta4(model)
            val rk45 = RungeKutta45(model)

            var t      = 0.0
            var x      = x0.copyOf()
            var stepDt = dt

            while (isActive && t < tEnd) {
                val u = controller(t, x)

                if (!useRK45) {
                    x  = rk4.step(t, x, u, dt)
                    t += dt
                } else {
                    var result = rk45.step(t, x, u, stepDt)
                    while (!result.accepted) {
                        result = rk45.step(t, x, u, result.dtNext)
                    }
                    stepDt = result.dtNext
                    t      = result.tNew
                    x      = result.xNew
                }

                val line = logger.log(t, x, u)
                onLogLine?.let { withContext(Dispatchers.Main) { it(line) } }
            }
            logger.close()
        }
    }

    fun stop() = job?.cancel()
}
