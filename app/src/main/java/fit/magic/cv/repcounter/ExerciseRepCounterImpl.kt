// Copyright (c) 2024 Magic Tech Ltd

package fit.magic.cv.repcounter

import fit.magic.cv.PoseLandmarkerHelper
import kotlin.math.acos
import kotlin.math.sqrt

class ExerciseRepCounterImpl : ExerciseRepCounter() {

    /** Main Task: Rep detection and progress tracking **/
    // Calculate the hip angle using the dot product and apply a median filter to smooth the values
    // Define the angle boundaries for the progress bar display
    // Set threshold values to distinguish between a lunge and a standing pose

    // Variables for median filter
    // Frame counters and filtering settings
    private var frameNum = 0
    private var filterFrameNum = 5

    // Arrays to cache joint rotation and distance values over frames
    private var lHipRotArray = FloatArray(filterFrameNum)
    private var rHipRotArray = FloatArray(filterFrameNum)
    private var lKneeRotArray = FloatArray(filterFrameNum)
    private var rKneeRotArray = FloatArray(filterFrameNum)
    private var ankleDistArray = FloatArray(filterFrameNum)

    // Variables for implementing progress bar
    // Boundary angles for hip rotation to prevent over-extension and over-flexion
    val maxHipRot = 150f
    val minHipRot = 90f

    // Variables for rep counter
    // Threshold for counting a repetition
    private val repThreshold = 0.6f
    private val readyPoseThreshold = 0.2f
    private var isReadyLunge: Boolean = true

    /** Bonus Task 1: Detecting the Lunge Side **/
    // Identify the forward leg by finding the one with the minimum hip angle
    // Use isSideUpdate to ensure detection occurs only once per repetition

    // Variables for determining which leg is performing the movement
    // 1: right leg; -1: left leg; 0: no movement detected
    private var previousLegSide: Int = 0
    private var currentLegSide: Int = 0
    private var isSideUpdate: Boolean = false

    /** Bonus Task 2: Knee Safety **/
    // Use a threshold to check if the forward knee is in a safe range
    // Set a threshold for the forward knee angle to ensure proper form and knee safety
    private val fowardKneeAngleThresh = 60f
    private var minFowardKneeAngle = 180f

    /** Bonus Task 3: Lunge vs Squat Detection **/
    // Compare the maximum ankle distance during the lunge pose
    // with the minimum ankle distance during the standing pose
    // Set a threshold for ankle distance to distinguish between movements
    // Perform the check after the first repetition
    private val fixFootThresh = 0.15f
    // Variables for tracking maximum and minimum distance between ankles
    private var maxAnkleDist = 0f
    private var minAnkleDist = 2f
    // Rep count
    private var repCount = 0

    // Define indices for different joints
    val lShoulderIdx = 11
    val rShoulderIdx = 12
    val lHipIdx = 23
    val rHipIdx = 24
    val lKneeIdx = 25
    val rKneeIdx = 26
    val lAnkleIdx = 27
    val rAnkleIdx = 28

    // Feedback message after each rep
    var feedbackMsg = ""

    // Function to calculate the distance between two points p0, p1
    private fun calculateDist(
        resultBundle: PoseLandmarkerHelper.ResultBundle,
        p0Index: Int,
        p1Index: Int,
    ): Float {
        val worldPoses = resultBundle.results[0].worldLandmarks()

        // Extract landmarks at the specified indices
        val p0 = worldPoses[0][p0Index]
        val p1 = worldPoses[0][p1Index]

        // Calculate vectors p0p1 and p0p2
        val v0p1x = p1.x() - p0.x()
        val v0p1y = p1.y() - p0.y()
        val v0p1z = p1.z() - p0.z()

        // Calculate the magnitudes of vectors p0p1 and p0p2
        val magnitudeP0P1 = sqrt(v0p1x * v0p1x + v0p1y * v0p1y + v0p1z * v0p1z)

        return magnitudeP0P1
    }

    // Function to calculate the angle between vectors formed by points p0, p1, and p2
    private fun calculateAngle(
        resultBundle: PoseLandmarkerHelper.ResultBundle,
        p0Index: Int,
        p1Index: Int,
        p2Index: Int
    ): Float {
        val worldPoses = resultBundle.results[0].worldLandmarks()

        // Extract landmarks at the specified indices
        val p0 = worldPoses[0][p0Index]
        val p1 = worldPoses[0][p1Index]
        val p2 = worldPoses[0][p2Index]

        // Calculate vectors p0p1 and p0p2
        val v0p1x = p1.x() - p0.x()
        val v0p1y = p1.y() - p0.y()
        val v0p1z = p1.z() - p0.z()

        val v0p2x = p2.x() - p0.x()
        val v0p2y = p2.y() - p0.y()
        val v0p2z = p2.z() - p0.z()

        // Calculate the dot product of vectors p0p1 and p0p2
        val dotProduct = v0p1x * v0p2x + v0p1y * v0p2y + v0p1z * v0p2z

        // Calculate the magnitudes of vectors p0p1 and p0p2
        val magnitudeP0P1 = calculateDist(resultBundle, p0Index, p1Index)
        val magnitudeP0P2 = calculateDist(resultBundle, p0Index, p2Index)

        // Calculate the angle in radians and convert it to degrees
        val angleInRadians = acos(dotProduct / (magnitudeP0P1 * magnitudeP0P2)).toDouble()
        val angleInDegrees = Math.toDegrees(angleInRadians).toFloat()
        return angleInDegrees
    }

    // Function to get median of an Array
    fun median(arr: FloatArray): Float {
        val sortedArr = arr.sorted()  // Sort the array

        val middle = sortedArr.size / 2
        return if (sortedArr.size % 2 == 0) {
            // If even, return the average of the two middle elements
            (sortedArr[middle - 1] + sortedArr[middle]) / 2
        } else {
            // If odd, return the middle element
            sortedArr[middle]
        }
    }

    override fun setResults(resultBundle: PoseLandmarkerHelper.ResultBundle) {
        // process pose data in resultBundle
        //
        // use functions in base class incrementRepCount(), sendProgressUpdate(),
        // and sendFeedbackMessage() to update the UI

        // get world coordinate to avoid scale issues of wide lens
        val worldPoses = resultBundle.results[0].worldLandmarks()

        // Check if pose data exists
        if (worldPoses.size > 0) {
            // Update joint rotation values and distance in arrays for current frame
            val rotArrayIdx = frameNum % filterFrameNum
            lHipRotArray[rotArrayIdx] = calculateAngle(resultBundle, lHipIdx, lShoulderIdx, lKneeIdx)
            rHipRotArray[rotArrayIdx] = calculateAngle(resultBundle, rHipIdx, rShoulderIdx, rKneeIdx)
            lKneeRotArray[rotArrayIdx] = calculateAngle(resultBundle, lKneeIdx, lShoulderIdx, lAnkleIdx)
            rKneeRotArray[rotArrayIdx] = calculateAngle(resultBundle, rKneeIdx, rShoulderIdx, rAnkleIdx)
            ankleDistArray[rotArrayIdx] = calculateDist(resultBundle, lAnkleIdx, rAnkleIdx)

            frameNum += 1

            // Only process data once the minimum required frames are collected
            if (frameNum >= filterFrameNum){
                val lHipRotMedian = median(lHipRotArray)
                val rHipRotMedian = median(rHipRotArray)
                val lKneeRotMedian = median(lKneeRotArray)
                val rKneeRotMedian = median(rKneeRotArray)
                val ankleDistMedian = median(ankleDistArray)

                // Calculate exercise progress based on hip rotation
                val minRotation = minOf(lHipRotMedian, rHipRotMedian)
                val progressUpdate = 1 - (minRotation - minHipRot) / (maxHipRot - minHipRot)
                val progressUpdateCorrect = progressUpdate.coerceIn(0f, 1f)
                sendProgressUpdate(progressUpdateCorrect)

                // Increment rep count if threshold is reached and is ready for a new rep
                if (progressUpdateCorrect > repThreshold) {
                    if (isReadyLunge){
                        incrementRepCount()
                        repCount++

                        isReadyLunge = false

                        feedbackMsg = ""

                        // Check the step size to determine whwether lunge or squat perform
                        if (repCount > 1){
                            // Update max ankle distances for squat/lunge detection
                            maxAnkleDist = maxOf(maxAnkleDist, ankleDistMedian)
                            println("maxAnkleDist - minAnkleDist: $maxAnkleDist - $minAnkleDist")
                            if ((maxAnkleDist - minAnkleDist) < fixFootThresh) {
                                feedbackMsg = "squat? move your foot\n"
                            }

                            // Reset distance variables
                            maxAnkleDist = 0f
                            minAnkleDist = 2f
                        }


                        // Check if the leg performing the exercise has switched
                        if (!isSideUpdate){
                            currentLegSide = if (lHipRotMedian > rHipRotMedian) -1 else 1
                            if (currentLegSide * previousLegSide <= 0) {
                                previousLegSide = currentLegSide
                            } else {
                                feedbackMsg += "switch leg"
                            }
                            sendFeedbackMessage(feedbackMsg)
                            isSideUpdate = true
                        }
                    }
                    // Track minimum knee angle for forward knee safety
                    if (currentLegSide == -1) {
                        minFowardKneeAngle = minOf(minFowardKneeAngle, lKneeRotMedian)
                    } else {
                        minFowardKneeAngle = minOf(minFowardKneeAngle, rKneeRotMedian)
                    }
                }else if (progressUpdateCorrect < readyPoseThreshold){
                    minAnkleDist = minOf(minAnkleDist, ankleDistMedian)
                    // Reset for the next rep and provide feedback on posture
                    // Update min ankle distances for squat/lunge detection

                    if (!isReadyLunge){
                        feedbackMsg = ""
                        if (minFowardKneeAngle < fowardKneeAngleThresh) {
                            feedbackMsg = "knee behind your toes \n"
                        }
                    }

                    sendFeedbackMessage(feedbackMsg)
                    isReadyLunge = true
                    isSideUpdate = false
                }

            }

        } else {
            println("No pose results found.")
        }

    }
}
