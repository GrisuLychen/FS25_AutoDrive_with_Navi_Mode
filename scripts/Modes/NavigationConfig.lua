ADNavigationConfig = ADNavigationConfig or {
    -- update cadence
    recalculateIntervalMs = 1400,
    analysisIntervalMs = 120,
    progressSampleMs = 450,

    -- ui / cueing
    maneuverDisplayRange = 200,
    showSecondaryManeuver = false,
    suppressSecondaryStraight = true,

    -- maneuver detection at intersections
    approachLookDistance = 24,
    exitLookDistance = 38,
    branchLookDistance = 38,
    straightCandidateAngle = 26,
    straightAngle = 18,
    sharpTurnAngle = 135,

    -- active maneuver lock / completion
    decisionZoneDistance = 160,
    intersectionExitLockDistance = 48,
    softRefreshBlockDistance = 320,
    approachPointAheadMargin = 0.15,
    maneuverDistanceDirectGuardRatio = 0.35,
    maneuverPassedMargin = 2,
    decisionHoldAfterExit = 28,
    activeManeuverExitHold = 26,
    passedSegmentBuffer = 2,
    maneuverNodeReachDistance = 14,
    maneuverNodeCaptureDistance = 28,
    maneuverNodeReleaseDistance = 9,
    maneuverNodeProjectionConfirmDistance = 18,
    maneuverNodeRouteMargin = 0.8,
    exitPassedRadius = 12,
    exitCaptureDistance = 30,
    exitReleaseDistance = 10,
    hardPassedDistance = 22,

    -- projection stability while a maneuver is active
    projectionJumpTolerance = 22,
    projectionMinAdvancePerTick = 2.5,
    projectionAdvanceFactor = 2.4,
    projectionAdvanceBuffer = 1.0,
    projectionMaxRetreatPerTick = 1.2,
    projectionNodeOvershoot = 1.2,
    projectionJumpPenalty = 0.45,
    projectionLockPastExitBuffer = 5,
    projectionLockPenalty = 2.1,
    projectionUnlockNodeDistance = 18,
    projectionUnlockRouteMargin = 1.5,

    -- reroute hysteresis
    rerouteDistanceBase = 60,
    rerouteDistanceIntersection = 125,
    rerouteDistanceIntersectionHard = 190,
    rerouteDelayMs = 11000,

    -- wrong-way detection
    wrongWayDelayMs = 10500,
    wrongWayHeadingDeg = 150,
    wrongWayMinSpeedKmh = 9,
    wrongWayMaxRouteDistance = 12,
    wrongWayMaxRouteDistanceIntersection = 18,
    wrongWayReverseProgressMs = 1100,

    -- route lookahead for current heading comparison
    routeHeadingLookahead = 30,

    -- bar reference distances
    barRefSharp = 55,
    barRefSlow = 50,
    barRefMedium = 65,
    barRefFast = 85,
    barRefVeryFast = 105,
    barRefMax = 120,

    -- distance display smoothing
    distanceMinDropPerTick = 1.5,
    distanceDropFactor = 1.7,
    distanceDropBuffer = 0.8,
    distanceMaxRisePerTick = 3.5,
    distanceSameManeuverMaxRisePerTick = 0.05,
}
