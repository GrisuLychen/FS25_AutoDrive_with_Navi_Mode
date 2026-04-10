NavigationMode = ADInheritsFrom(AbstractMode)

local function navCfg(key, default)
    if ADNavigationConfig ~= nil and ADNavigationConfig[key] ~= nil then
        return ADNavigationConfig[key]
    end
    return default
end

function NavigationMode:new(vehicle)
    local o = NavigationMode:create()
    o.vehicle = vehicle
    NavigationMode.reset(o)
    return o
end

function NavigationMode:reset()
    self.recalculateTimer = 0
    self.analysisTimer = 0
    self.recalculateInterval = navCfg("recalculateIntervalMs", 1400)
    self.analysisInterval = navCfg("analysisIntervalMs", 120)
    self.distanceToTurn = nil
    self.directDistanceToTurn = nil
    self.displayDistanceToTurn = nil
    self.totalDistance = nil
    self.turnType = "noRoute"
    self.targetName = ""
    self.nextTurnType = nil
    self.nextDistanceToTurn = nil
    self.distanceBarText = ""
    self.lastRawDistanceToTurn = nil
    self.route = nil
    self.routeCumulative = nil
    self.routeTotalDistance = 0
    self.routeTargetId = nil
    self.routeProjection = nil
    self.lockedTurnPointId = nil
    self.lockedTurnType = nil
    self.lockedTurnDistanceAlong = nil
    self.lockedTurnRouteIndex = nil
    self.lockedTurnExitDistanceAlong = nil
    self.lockedTurnExitRouteIndex = nil
    self.lockedTurnRecommendedSpeed = nil
    self.lockedTurnPoint = nil
    self.lockedTurnExitPoint = nil
    self.cueTurnPointId = nil
    self.cueTurnType = nil
    self.cueTurnDistanceAlong = nil
    self.barDistanceToTurn = nil
    self.offRouteTimer = 0
    self.wrongWayTimer = 0
    self.routeInvalidTimer = 0
    self.progressSampleTimer = 0
    self.lastProgressDistanceAlong = nil
    self.lastProgressDelta = 0
    self.reverseProgressTimer = 0
    self.forwardProgressTimer = 0
    self.maneuverDisplayRange = navCfg("maneuverDisplayRange", 200)
    self.stableProjectionAlong = nil
    self.lastAnalysisDt = self.analysisInterval
    self.lockedTurnMinPointDistance = nil
    self.lockedTurnMinExitDistance = nil
    self.activeDisplayManeuverKey = nil
    self.activeDisplayRawMin = nil
end

function NavigationMode:start(user)
    if self.vehicle.ad.stateModule:getFirstMarker() == nil then
        return
    end

    self:reset()
    self.targetName = self.vehicle.ad.stateModule:getFirstMarker().name or ""
    self.vehicle.ad.stateModule:setNavigationActive(true)
    self:refreshRoute(true)
    self:update(self.analysisInterval)
end

function NavigationMode:stop()
    self.vehicle.ad.stateModule:setNavigationActive(false)
    self:reset()
end

function NavigationMode:update(dt)
    if not self.vehicle.ad.stateModule:isNavigationActive() then
        return
    end

    self.analysisTimer = self.analysisTimer + dt
    self.recalculateTimer = self.recalculateTimer + dt

    local firstMarker = self.vehicle.ad.stateModule:getFirstMarker()
    if firstMarker == nil or firstMarker.id == nil then
        self:clearDisplay("noRoute")
        return
    end

    self.targetName = firstMarker.name or ""

    local needsRefresh = self.route == nil or self.routeTargetId ~= firstMarker.id
    local canSoftRefresh = self.recalculateTimer >= self.recalculateInterval and self:shouldSoftRefreshRoute()
    if needsRefresh or canSoftRefresh then
        self:refreshRoute(false)
    end

    if self.analysisTimer < self.analysisInterval then
        return
    end
    self.analysisTimer = 0

    local info = self:analyzeCurrentRoute(dt)

    if info ~= nil then
        local timerStep = math.max(dt, self.analysisInterval)
        if info.isFarFromRoute then
            self.offRouteTimer = math.min(6000, self.offRouteTimer + timerStep)
        else
            self.offRouteTimer = math.max(0, self.offRouteTimer - timerStep * 2)
        end

        if info.isWrongWay then
            self.wrongWayTimer = math.min(6000, self.wrongWayTimer + timerStep)
        else
            self.wrongWayTimer = math.max(0, self.wrongWayTimer - timerStep * 2)
        end

        if info.allowRecalculation and info.isFarFromRoute and self.offRouteTimer >= navCfg("rerouteDelayMs", 2600) then
            self:refreshRoute(true)
            info = self:analyzeCurrentRoute(dt)
        elseif info.allowRecalculation and info.isWrongWay and self.wrongWayTimer >= navCfg("wrongWayDelayMs", 3200) then
            self:refreshRoute(true)
            info = self:analyzeCurrentRoute(dt)
        end
    end

    self:applyInfo(info)
end

function NavigationMode:shouldSoftRefreshRoute()
    if self.route == nil or self.routeProjection == nil then
        return true
    end

    if self.lockedTurnPointId == nil or self.lockedTurnDistanceAlong == nil then
        return true
    end

    local projectionAlong = self:getProjectionDistanceAlong(self.routeProjection)
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local activeManeuver = {
        pointId = self.lockedTurnPointId,
        turnType = self.lockedTurnType,
        distanceAlongRoute = self.lockedTurnDistanceAlong,
        routeIndex = self.lockedTurnRouteIndex,
        exitDistanceAlong = self.lockedTurnExitDistanceAlong,
        exitRouteIndex = self.lockedTurnExitRouteIndex,
        point = self.lockedTurnPoint,
        exitPoint = self.lockedTurnExitPoint
    }

    local stillActive = not self:isManeuverPassed(activeManeuver, self.routeProjection, x, z)
    local distanceToNode = math.max(0, (self.lockedTurnDistanceAlong or 0) - projectionAlong)
    local softRefreshBlockDistance = navCfg("softRefreshBlockDistance", 260)
    if stillActive and distanceToNode <= softRefreshBlockDistance then
        return false
    end

    return true
end

function NavigationMode:clearDisplay(turnType)
    self.turnType = turnType or "noRoute"
    self.distanceToTurn = nil
    self.directDistanceToTurn = nil
    self.displayDistanceToTurn = nil
    self.totalDistance = nil
    self.nextTurnType = nil
    self.nextDistanceToTurn = nil
    self.distanceBarText = ""
    self.lastRawDistanceToTurn = nil
    self.route = nil
    self.routeCumulative = nil
    self.routeTotalDistance = 0
    self.routeProjection = nil
    self.lockedTurnPointId = nil
    self.lockedTurnType = nil
    self.lockedTurnDistanceAlong = nil
    self.lockedTurnRouteIndex = nil
    self.lockedTurnExitDistanceAlong = nil
    self.lockedTurnExitRouteIndex = nil
    self.lockedTurnRecommendedSpeed = nil
    self.lockedTurnPoint = nil
    self.lockedTurnExitPoint = nil
    self.cueTurnPointId = nil
    self.cueTurnType = nil
    self.cueTurnDistanceAlong = nil
    self.barDistanceToTurn = nil
    self.progressSampleTimer = 0
    self.lastProgressDistanceAlong = nil
    self.lastProgressDelta = 0
    self.reverseProgressTimer = 0
    self.forwardProgressTimer = 0
    self.stableProjectionAlong = nil
    self.lastAnalysisDt = self.analysisInterval
    self.lockedTurnMinPointDistance = nil
    self.lockedTurnMinExitDistance = nil
    self.activeDisplayManeuverKey = nil
    self.activeDisplayRawMin = nil
end

function NavigationMode:refreshRoute(force)
    local firstMarker = self.vehicle.ad.stateModule:getFirstMarker()
    if firstMarker == nil or firstMarker.id == nil then
        self:clearDisplay("noRoute")
        return
    end

    self.recalculateTimer = 0
    self.routeTargetId = firstMarker.id

    local route = ADGraphManager:getPathTo(self.vehicle, firstMarker.id)
    if route == nil or #route == 0 then
        self.route = nil
        self.routeCumulative = nil
        self.routeTotalDistance = 0
        self.routeProjection = nil
        return
    end

    self.route = route
    self.routeCumulative = self:buildCumulativeDistances(route)
    self.routeTotalDistance = self.routeCumulative[#self.routeCumulative] or 0
    self.routeProjection = nil

    if force then
        self.lockedTurnPointId = nil
        self.lockedTurnType = nil
        self.lockedTurnDistanceAlong = nil
        self.lockedTurnRouteIndex = nil
        self.lockedTurnExitDistanceAlong = nil
        self.lockedTurnExitRouteIndex = nil
        self.lockedTurnRecommendedSpeed = nil
        self.lockedTurnPoint = nil
        self.lockedTurnExitPoint = nil
        self.cueTurnPointId = nil
        self.cueTurnType = nil
        self.cueTurnDistanceAlong = nil
        self.progressSampleTimer = 0
        self.lastProgressDistanceAlong = nil
        self.lastProgressDelta = 0
        self.reverseProgressTimer = 0
        self.forwardProgressTimer = 0
        self.stableProjectionAlong = nil
        self.lockedTurnMinPointDistance = nil
        self.lockedTurnMinExitDistance = nil
        self.activeDisplayManeuverKey = nil
        self.activeDisplayRawMin = nil
    end
end

function NavigationMode:buildCumulativeDistances(route)
    local cumulative = {0}
    local distance = 0

    for index = 1, #route - 1 do
        local pointA = route[index]
        local pointB = route[index + 1]
        distance = distance + MathUtil.vector2Length(pointA.x - pointB.x, pointA.z - pointB.z)
        cumulative[index + 1] = distance
    end

    return cumulative
end

function NavigationMode:analyzeCurrentRoute(dt)
    local firstMarker = self.vehicle.ad.stateModule:getFirstMarker()
    if firstMarker == nil or firstMarker.id == nil then
        return {
            turnType = "noRoute"
        }
    end

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)

    if self.route == nil or #self.route == 0 then
        local directDistance = self:getDirectDistanceToTarget(firstMarker.id)
        return {
            turnType = "noRoute",
            distanceToTurn = directDistance,
            directDistanceToTurn = directDistance,
            totalDistance = directDistance,
            nextTurnType = nil,
            nextDistanceToTurn = nil,
            projection = nil,
            isFarFromRoute = false,
            isWrongWay = false,
            distanceBarText = "",
            allowRecalculation = true
        }
    end

    if #self.route == 1 then
        local directDistance = MathUtil.vector2Length(x - self.route[1].x, z - self.route[1].z)
        return {
            turnType = "arrived",
            distanceToTurn = directDistance,
            directDistanceToTurn = directDistance,
            totalDistance = directDistance,
            nextTurnType = nil,
            nextDistanceToTurn = nil,
            projection = nil,
            isFarFromRoute = false,
            isWrongWay = false,
            distanceBarText = "",
            allowRecalculation = true
        }
    end

    local vehicleSpeed = self:getVehicleSpeedKmh()
    local projection = self:getRouteProjection(self.route, x, z)
    projection = self:stabilizeProjection(projection, dt, vehicleSpeed, x, z)
    self.routeProjection = projection

    local projectionDistanceAlong = self:getProjectionDistanceAlong(projection)
    local totalDistance = math.max(0, self.routeTotalDistance - projectionDistanceAlong)
    local maneuvers = self:collectUpcomingRouteDecisions(self.route, projection)
    local primary, secondary = self:selectManeuvers(maneuvers, projection, x, z)
    local cueManeuver = primary

    local primaryDistance = self:getActiveManeuverDisplayDistance(primary, projection, x, z)
    local cueDistance = primaryDistance
    local headingDelta = self:getHeadingDelta(self.route, projection)
    self:updateProgressState(projection, dt, vehicleSpeed)

    local decisionZoneDistance = navCfg("decisionZoneDistance", 160)
    local decisionHoldAfterExit = navCfg("decisionHoldAfterExit", 24)
    local inDecisionZone = false
    if primary ~= nil then
        local exitDistanceAlong = primary.exitDistanceAlong or primary.distanceAlongRoute
        inDecisionZone = (primaryDistance ~= nil and primaryDistance <= decisionZoneDistance)
            or (projectionDistanceAlong <= (exitDistanceAlong + decisionHoldAfterExit))
    end

    local rerouteDistanceThreshold = inDecisionZone and navCfg("rerouteDistanceIntersection", 85) or navCfg("rerouteDistanceBase", 45)
    local hardIntersectionThreshold = navCfg("rerouteDistanceIntersectionHard", 130)
    local isFarFromRoute = projection.distanceToRoute >= rerouteDistanceThreshold
    local isWrongWay = false
    if not inDecisionZone then
        isWrongWay = self:isVehicleWrongWay(headingDelta, projection, totalDistance, vehicleSpeed, false)
    elseif projection.distanceToRoute >= hardIntersectionThreshold then
        isWrongWay = self:isVehicleWrongWay(headingDelta, projection, totalDistance, vehicleSpeed, true)
    end

    local allowRecalculation = true
    if inDecisionZone then
        allowRecalculation = projection.distanceToRoute >= hardIntersectionThreshold
    end

    local activeManeuverHold = false
    if primary ~= nil and not self:isManeuverPassed(primary, projection, x, z) then
        local exitDistanceAlong = primary.exitDistanceAlong or primary.distanceAlongRoute
        activeManeuverHold = projectionDistanceAlong <= (exitDistanceAlong + navCfg("activeManeuverExitHold", 20))
    end
    if activeManeuverHold and projection.distanceToRoute < navCfg("rerouteDistanceIntersectionHard", 130) then
        allowRecalculation = false
    end

    local result = {
        turnType = "straight",
        distanceToTurn = nil,
        directDistanceToTurn = nil,
        totalDistance = totalDistance,
        nextTurnType = nil,
        nextDistanceToTurn = nil,
        barDistanceToTurn = nil,
        projection = projection,
        isFarFromRoute = isFarFromRoute,
        isWrongWay = isWrongWay,
        distanceBarText = "",
        allowRecalculation = allowRecalculation,
        analysisDt = dt,
        maneuverKey = nil,
        rawManeuverDistance = nil
    }

    if primary ~= nil then
        local shouldShowManeuver = primaryDistance ~= nil and primaryDistance <= self.maneuverDisplayRange
        local maneuverReached = self:hasReachedManeuverPoint(primary, projection, x, z)
        if shouldShowManeuver or maneuverReached then
            result.turnType = primary.turnType
            result.distanceToTurn = primaryDistance
            result.directDistanceToTurn = primaryDistance
            result.barDistanceToTurn = cueDistance
            result.distanceBarText = self:getDistanceBarText(cueDistance, primary.recommendedSpeed, primary.turnType)
            result.maneuverKey = string.format("%s:%s", tostring(primary.pointId), tostring(primary.turnType))
            result.rawManeuverDistance = primaryDistance
        end
    elseif totalDistance <= 12 then
        result.turnType = "arrived"
        result.distanceToTurn = totalDistance
        result.directDistanceToTurn = totalDistance
    end

    if isWrongWay and self.wrongWayTimer >= 900 then
        result.turnType = "uturn"
        result.distanceToTurn = nil
        result.directDistanceToTurn = nil
        result.barDistanceToTurn = nil
        result.distanceBarText = ""
        result.nextTurnType = nil
        result.nextDistanceToTurn = nil
    end

    return result
end

function NavigationMode:applyInfo(info)
    if info == nil then
        self:clearDisplay("noRoute")
        return
    end

    local previousTurnType = self.turnType
    self.routeProjection = info.projection or self.routeProjection
    self.totalDistance = info.totalDistance
    self.nextTurnType = info.nextTurnType
    self.nextDistanceToTurn = info.nextDistanceToTurn
    self.distanceToTurn = info.distanceToTurn
    self.directDistanceToTurn = info.directDistanceToTurn
    self.lastRawDistanceToTurn = info.distanceToTurn
    self.turnType = info.turnType or "straight"
    self.distanceBarText = info.distanceBarText or ""
    self.barDistanceToTurn = info.barDistanceToTurn

    if self.turnType == "uturn" then
        self.displayDistanceToTurn = nil
        return
    end

    self.lastAnalysisDt = info.analysisDt or self.analysisInterval
    self.displayDistanceToTurn = self:smoothDistance(info.distanceToTurn, info.turnType, previousTurnType, info.isFarFromRoute, info.maneuverKey)
end

function NavigationMode:getDirectDistanceToTarget(wayPointId)
    local target = ADGraphManager:getWayPointById(wayPointId)
    if target == nil then
        return nil
    end

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    return MathUtil.vector2Length(x - target.x, z - target.z)
end

function NavigationMode:getDistanceToRoutePoint(point)
    if point == nil then
        return nil
    end

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    return MathUtil.vector2Length(x - point.x, z - point.z)
end

function NavigationMode:getProjectionDistanceAlong(projection)
    if projection == nil then
        return 0
    end
    return projection.stableDistanceAlongRoute or projection.distanceAlongRoute or 0
end

function NavigationMode:stabilizeProjection(projection, dt, vehicleSpeed, vehicleX, vehicleZ)
    if projection == nil then
        self.stableProjectionAlong = nil
        return nil
    end

    local rawAlong = projection.distanceAlongRoute or 0
    local stableAlong = rawAlong
    local dtMs = math.max(dt or self.analysisInterval or 120, 60)
    local speedMs = math.max(0, vehicleSpeed or 0) / 3.6
    local maxAdvance = math.max(navCfg("projectionMinAdvancePerTick", 2.5), speedMs * (dtMs / 1000) * navCfg("projectionAdvanceFactor", 2.4) + navCfg("projectionAdvanceBuffer", 1.0))
    local maxRetreat = navCfg("projectionMaxRetreatPerTick", 1.2)

    if self.stableProjectionAlong ~= nil then
        stableAlong = math.max(self.stableProjectionAlong - maxRetreat, math.min(stableAlong, self.stableProjectionAlong + maxAdvance))
    end

    if self.lockedTurnDistanceAlong ~= nil then
        local pointDistance = nil
        if self.lockedTurnPoint ~= nil and vehicleX ~= nil and vehicleZ ~= nil then
            pointDistance = MathUtil.vector2Length(vehicleX - self.lockedTurnPoint.x, vehicleZ - self.lockedTurnPoint.z)
        end
        local unlockDistance = navCfg("projectionUnlockNodeDistance", 18)
        local overshoot = navCfg("projectionNodeOvershoot", 1.2)
        local nodeReached = false
        if pointDistance ~= nil and pointDistance <= unlockDistance then
            nodeReached = true
        end
        if not nodeReached and stableAlong > (self.lockedTurnDistanceAlong + overshoot) then
            stableAlong = self.lockedTurnDistanceAlong + overshoot
        end
    end

    projection.stableDistanceAlongRoute = stableAlong
    self.stableProjectionAlong = stableAlong
    return projection
end

function NavigationMode:getRouteProjection(route, vehicleX, vehicleZ)
    local best = {
        distanceSq = math.huge,
        distanceAlongRoute = 0,
        distanceToRoute = 0,
        segmentIndex = 1,
        t = 0,
        x = route[1].x,
        y = route[1].y,
        z = route[1].z
    }

    local segmentCount = math.max(1, #route - 1)
    local startIndex = 1
    local endIndex = math.min(segmentCount, 80)

    if self.routeProjection ~= nil and self.routeProjection.segmentIndex ~= nil then
        startIndex = math.max(1, self.routeProjection.segmentIndex - 5)
        endIndex = math.min(segmentCount, self.routeProjection.segmentIndex + 26)
    end

    for index = startIndex, endIndex do
        local pointA = route[index]
        local pointB = route[index + 1]
        local segmentX = pointB.x - pointA.x
        local segmentZ = pointB.z - pointA.z
        local segmentLengthSq = (segmentX * segmentX) + (segmentZ * segmentZ)

        if segmentLengthSq > 0.0001 then
            local t = (((vehicleX - pointA.x) * segmentX) + ((vehicleZ - pointA.z) * segmentZ)) / segmentLengthSq
            t = math.max(0, math.min(1, t))

            local projX = pointA.x + segmentX * t
            local projY = pointA.y + (pointB.y - pointA.y) * t
            local projZ = pointA.z + segmentZ * t
            local distX = vehicleX - projX
            local distZ = vehicleZ - projZ
            local distSq = (distX * distX) + (distZ * distZ)

            local segmentLength = math.sqrt(segmentLengthSq)
            local distanceAlong = (self.routeCumulative[index] or 0) + segmentLength * t
            local cost = distSq

            if self.routeProjection ~= nil and self.routeProjection.distanceAlongRoute ~= nil then
                local previousAlong = self.routeProjection.distanceAlongRoute
                local jump = math.abs(distanceAlong - previousAlong)
                local jumpTolerance = navCfg("projectionJumpTolerance", 22)
                if jump > jumpTolerance then
                    local penalty = jump - jumpTolerance
                    cost = cost + penalty * penalty * navCfg("projectionJumpPenalty", 0.45)
                end
            end

            if self.lockedTurnExitDistanceAlong ~= nil and self.routeProjection ~= nil and self.routeProjection.distanceAlongRoute ~= nil then
                local lockPastExit = true
                if self.lockedTurnPoint ~= nil then
                    local distToLockedTurn = MathUtil.vector2Length(vehicleX - self.lockedTurnPoint.x, vehicleZ - self.lockedTurnPoint.z)
                    if distToLockedTurn <= navCfg("projectionUnlockNodeDistance", 18) then
                        lockPastExit = false
                    end
                end
                if self.lockedTurnDistanceAlong ~= nil and self.routeProjection.distanceAlongRoute >= (self.lockedTurnDistanceAlong - navCfg("projectionUnlockRouteMargin", 1.5)) then
                    lockPastExit = false
                end

                if lockPastExit then
                    local lockLimit = self.lockedTurnExitDistanceAlong + navCfg("projectionLockPastExitBuffer", 6)
                    if self.routeProjection.distanceAlongRoute < lockLimit and distanceAlong > lockLimit then
                        local penalty = distanceAlong - lockLimit
                        cost = cost + penalty * penalty * navCfg("projectionLockPenalty", 2.5)
                    end
                end
            end

            if cost < best.distanceSq then
                best.distanceSq = cost
                best.distanceAlongRoute = distanceAlong
                best.distanceToRoute = math.sqrt(distSq)
                best.segmentIndex = index
                best.t = t
                best.x = projX
                best.y = projY
                best.z = projZ
            end
        end
    end

    if best.distanceSq == math.huge then
        local firstPoint = route[1]
        best.distanceSq = (vehicleX - firstPoint.x) * (vehicleX - firstPoint.x) + (vehicleZ - firstPoint.z) * (vehicleZ - firstPoint.z)
        best.distanceAlongRoute = 0
        best.distanceToRoute = math.sqrt(best.distanceSq)
        best.segmentIndex = 1
        best.t = 0
        best.x = firstPoint.x
        best.y = firstPoint.y
        best.z = firstPoint.z
    end

    return best
end

function NavigationMode:collectUpcomingRouteDecisions(route, projection)
    local maneuvers = {}
    local startIndex = math.max(2, (projection.segmentIndex or 1) - 4)
    if self.lockedTurnRouteIndex ~= nil then
        startIndex = math.min(startIndex, math.max(2, self.lockedTurnRouteIndex - 2))
    end

    for routeIndex = startIndex, #route - 1 do
        local pointB = route[routeIndex]
        if self:isIntersectionNode(pointB) then
            local decision = self:evaluateRouteDecision(route, routeIndex)
            if decision ~= nil then
                local distanceAlongRoute = self.routeCumulative[routeIndex] or 0
                decision.routeIndex = routeIndex
                decision.pointId = pointB.id
                decision.point = pointB
                decision.distanceAlongRoute = distanceAlongRoute
                if not self:isManeuverPassed(decision, projection, nil, nil) then
                    table.insert(maneuvers, decision)
                end
            end
        end
    end

    return self:compressManeuvers(maneuvers)
end

function NavigationMode:evaluateRouteDecision(route, routeIndex)
    local pointA = route[routeIndex - 1]
    local pointB = route[routeIndex]
    local pointC = route[routeIndex + 1]
    if pointA == nil or pointB == nil or pointC == nil then
        return nil
    end

    local approachDirX, approachDirZ = self:getSmoothedRouteDirection(route, routeIndex, false, navCfg("approachLookDistance", 24))
    local exitDirX, exitDirZ = self:getSmoothedRouteDirection(route, routeIndex, true, navCfg("exitLookDistance", 38))
    if approachDirX == nil or exitDirX == nil then
        return nil
    end

    local chosenOutId = pointC.id
    local chosenAngle = self:getSignedAngleDeg(approachDirX, approachDirZ, exitDirX, exitDirZ)
    local straightCandidateId = nil
    local straightCandidateAbsAngle = math.huge

    if pointB.out ~= nil then
        for _, outId in pairs(pointB.out) do
            local branchDirX, branchDirZ = self:getSmoothedBranchDirection(pointB, outId, navCfg("branchLookDistance", 38))
            if branchDirX ~= nil then
                local branchAngle = self:getSignedAngleDeg(approachDirX, approachDirZ, branchDirX, branchDirZ)
                local branchAbsAngle = math.abs(branchAngle)
                if branchAbsAngle < straightCandidateAbsAngle then
                    straightCandidateAbsAngle = branchAbsAngle
                    straightCandidateId = outId
                end
                if outId == chosenOutId then
                    chosenAngle = branchAngle
                end
            end
        end
    end

    local turnType = self:getTurnTypeForAngle(chosenAngle, straightCandidateAbsAngle, straightCandidateId == chosenOutId)
    if turnType == nil or turnType == "straight" then
        return nil
    end

    local recommendedSpeed = self:getRecommendedTurnSpeed(math.abs(chosenAngle), turnType)
    local exitRouteIndex, exitDistanceAlong = self:getManeuverExit(route, routeIndex)
    local exitPoint = exitRouteIndex ~= nil and route[exitRouteIndex] or nil

    return {
        turnType = turnType,
        angle = math.abs(chosenAngle),
        signedAngle = chosenAngle,
        recommendedSpeed = recommendedSpeed,
        isIntersection = true,
        exitRouteIndex = exitRouteIndex,
        exitDistanceAlong = exitDistanceAlong,
        exitPoint = exitPoint
    }
end

function NavigationMode:getManeuverExit(route, routeIndex)
    if route == nil or routeIndex == nil then
        return nil, nil
    end

    local baseDistance = self.routeCumulative ~= nil and (self.routeCumulative[routeIndex] or 0) or 0
    local lockDistance = navCfg("intersectionExitLockDistance", 24)
    local exitIndex = math.min(routeIndex + 1, #route)
    local targetDistance = baseDistance + lockDistance

    for index = routeIndex + 1, #route do
        exitIndex = index
        local distanceAlong = self.routeCumulative ~= nil and (self.routeCumulative[index] or baseDistance) or baseDistance
        if distanceAlong >= targetDistance then
            break
        end
        if index > routeIndex + 1 and self:isIntersectionNode(route[index]) then
            exitIndex = math.max(routeIndex + 1, index - 1)
            break
        end
    end

    local exitDistanceAlong = self.routeCumulative ~= nil and (self.routeCumulative[exitIndex] or baseDistance) or baseDistance
    if exitDistanceAlong < baseDistance then
        exitDistanceAlong = baseDistance
    end

    return exitIndex, exitDistanceAlong
end

function NavigationMode:isIntersectionNode(point)
    if point == nil then
        return false
    end

    local outCount = point.out ~= nil and #point.out or 0
    local incomingCount = point.incoming ~= nil and #point.incoming or 0
    local unique = {}

    if point.out ~= nil then
        for _, outId in pairs(point.out) do
            unique[outId] = true
        end
    end
    if point.incoming ~= nil then
        for _, incomingId in pairs(point.incoming) do
            unique[incomingId] = true
        end
    end

    local uniqueCount = 0
    for _, _ in pairs(unique) do
        uniqueCount = uniqueCount + 1
    end

    if outCount > 1 then
        return true
    end
    if incomingCount > 1 and outCount > 0 then
        return true
    end
    if uniqueCount >= 3 then
        return true
    end

    return false
end

function NavigationMode:getTurnTypeForAngle(signedAngle, straightCandidateAbsAngle, isChosenStraightCandidate)
    local absAngle = math.abs(signedAngle)

    if isChosenStraightCandidate and straightCandidateAbsAngle <= navCfg("straightCandidateAngle", 26) and absAngle <= 32 then
        return "straight"
    end
    if absAngle <= navCfg("straightAngle", 18) then
        return "straight"
    end

    local direction = signedAngle < 0 and "left" or "right"
    if absAngle >= navCfg("sharpTurnAngle", 135) then
        return direction == "left" and "sharpLeft" or "sharpRight"
    end

    return direction
end

function NavigationMode:getRecommendedTurnSpeed(angle, turnType)
    if turnType == "straight" then
        return 42
    end

    local speed = 34

    if turnType == "sharpLeft" or turnType == "sharpRight" or angle >= 135 then
        speed = 10
    elseif angle >= 100 then
        speed = 14
    elseif angle >= 80 then
        speed = 18
    elseif angle >= 60 then
        speed = 24
    else
        speed = 28
    end

    return math.max(8, speed)
end

function NavigationMode:compressManeuvers(maneuvers)
    if maneuvers == nil or #maneuvers == 0 then
        return maneuvers
    end

    local compressed = {}
    for _, maneuver in ipairs(maneuvers) do
        local lastManeuver = compressed[#compressed]
        if lastManeuver ~= nil then
            local distanceDelta = maneuver.distanceAlongRoute - lastManeuver.distanceAlongRoute
            if maneuver.pointId == lastManeuver.pointId then
                lastManeuver.turnType = maneuver.turnType
                lastManeuver.angle = maneuver.angle
                lastManeuver.signedAngle = maneuver.signedAngle
                lastManeuver.recommendedSpeed = maneuver.recommendedSpeed
                lastManeuver.routeIndex = maneuver.routeIndex
                lastManeuver.distanceAlongRoute = maneuver.distanceAlongRoute
                lastManeuver.exitRouteIndex = maneuver.exitRouteIndex
                lastManeuver.exitDistanceAlong = maneuver.exitDistanceAlong
            elseif distanceDelta <= 8 and maneuver.turnType == lastManeuver.turnType then
                if maneuver.angle > lastManeuver.angle then
                    lastManeuver.turnType = maneuver.turnType
                    lastManeuver.angle = maneuver.angle
                    lastManeuver.signedAngle = maneuver.signedAngle
                    lastManeuver.recommendedSpeed = maneuver.recommendedSpeed
                    lastManeuver.routeIndex = maneuver.routeIndex
                    lastManeuver.pointId = maneuver.pointId
                    lastManeuver.point = maneuver.point
                    lastManeuver.isIntersection = maneuver.isIntersection
                    lastManeuver.distanceAlongRoute = maneuver.distanceAlongRoute
                    lastManeuver.exitRouteIndex = maneuver.exitRouteIndex
                    lastManeuver.exitDistanceAlong = maneuver.exitDistanceAlong
                end
            else
                table.insert(compressed, maneuver)
            end
        else
            table.insert(compressed, maneuver)
        end
    end

    return compressed
end

function NavigationMode:selectManeuvers(maneuvers, projection, vehicleX, vehicleZ)
    if maneuvers == nil then
        maneuvers = {}
    end

    local lockedManeuver = nil
    if self.lockedTurnPointId ~= nil then
        for _, maneuver in ipairs(maneuvers) do
            if maneuver.pointId == self.lockedTurnPointId and maneuver.turnType == self.lockedTurnType then
                if not self:isManeuverPassed(maneuver, projection, vehicleX, vehicleZ) then
                    lockedManeuver = maneuver
                end
                break
            end
        end

        if lockedManeuver == nil and self.lockedTurnPoint ~= nil and self.lockedTurnType ~= nil then
            local fallback = {
                pointId = self.lockedTurnPointId,
                turnType = self.lockedTurnType,
                distanceAlongRoute = self.lockedTurnDistanceAlong,
                routeIndex = self.lockedTurnRouteIndex,
                exitDistanceAlong = self.lockedTurnExitDistanceAlong,
                exitRouteIndex = self.lockedTurnExitRouteIndex,
                recommendedSpeed = self.lockedTurnRecommendedSpeed,
                point = self.lockedTurnPoint,
                exitPoint = self.lockedTurnExitPoint
            }
            if not self:isManeuverPassed(fallback, projection, vehicleX, vehicleZ) then
                lockedManeuver = fallback
            end
        end
    end

    local primary = lockedManeuver
    local secondary = nil

    if primary == nil then
        primary = maneuvers[1]
        secondary = maneuvers[2]
    else
        for index, maneuver in ipairs(maneuvers) do
            if maneuver.pointId == primary.pointId and maneuver.turnType == primary.turnType then
                secondary = maneuvers[index + 1]
                break
            end
        end
    end

    if primary == nil then
        self.lockedTurnPointId = nil
        self.lockedTurnType = nil
        self.lockedTurnDistanceAlong = nil
        self.lockedTurnRouteIndex = nil
        self.lockedTurnExitDistanceAlong = nil
        self.lockedTurnExitRouteIndex = nil
        self.lockedTurnRecommendedSpeed = nil
        self.lockedTurnPoint = nil
        self.lockedTurnExitPoint = nil
        self.lockedTurnMinPointDistance = nil
        self.lockedTurnMinExitDistance = nil
        return nil, nil
    end

    local previousLockedPointId = self.lockedTurnPointId
    local previousLockedType = self.lockedTurnType

    self.lockedTurnPointId = primary.pointId
    self.lockedTurnType = primary.turnType
    self.lockedTurnDistanceAlong = primary.distanceAlongRoute
    self.lockedTurnRouteIndex = primary.routeIndex
    self.lockedTurnExitDistanceAlong = primary.exitDistanceAlong
    self.lockedTurnExitRouteIndex = primary.exitRouteIndex
    self.lockedTurnRecommendedSpeed = primary.recommendedSpeed
    self.lockedTurnPoint = primary.point
    self.lockedTurnExitPoint = primary.exitPoint

    if previousLockedPointId ~= primary.pointId or previousLockedType ~= primary.turnType then
        self.lockedTurnMinPointDistance = nil
        self.lockedTurnMinExitDistance = nil
    end

    return primary, secondary
end

function NavigationMode:selectCueManeuver(maneuvers, projection)
    if maneuvers == nil or #maneuvers == 0 then
        self.cueTurnPointId = nil
        self.cueTurnType = nil
        self.cueTurnDistanceAlong = nil
        return nil
    end

    local cue = maneuvers[1]
    self.cueTurnPointId = cue.pointId
    self.cueTurnType = cue.turnType
    self.cueTurnDistanceAlong = cue.distanceAlongRoute
    return cue
end

function NavigationMode:isManeuverPassed(maneuver, projection, vehicleX, vehicleZ)
    if maneuver == nil or projection == nil then
        return true
    end

    local x, z = vehicleX, vehicleZ
    if x == nil or z == nil then
        x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    end

    local projectionAlong = self:getProjectionDistanceAlong(projection)
    local exitDistanceAlong = maneuver.exitDistanceAlong or maneuver.distanceAlongRoute
    local passedMargin = navCfg("maneuverPassedMargin", 2)
    local hardPastDistance = navCfg("hardPassedDistance", 22)
    local exitPassedRadius = navCfg("exitPassedRadius", 12)
    local exitCaptureDistance = navCfg("exitCaptureDistance", 24)
    local exitReleaseDistance = navCfg("exitReleaseDistance", 8)

    if maneuver.exitPoint ~= nil then
        local exitDirectDistance = MathUtil.vector2Length(x - maneuver.exitPoint.x, z - maneuver.exitPoint.z)
        if self.lockedTurnPointId == maneuver.pointId and self.lockedTurnType == maneuver.turnType then
            if self.lockedTurnMinExitDistance == nil or exitDirectDistance < self.lockedTurnMinExitDistance then
                self.lockedTurnMinExitDistance = exitDirectDistance
            end
        end
        if exitDirectDistance <= exitPassedRadius then
            return true
        end
        if self.lockedTurnPointId == maneuver.pointId and self.lockedTurnType == maneuver.turnType and self.lockedTurnMinExitDistance ~= nil then
            if self.lockedTurnMinExitDistance <= exitCaptureDistance and exitDirectDistance >= (self.lockedTurnMinExitDistance + exitReleaseDistance) then
                return true
            end
        end
    end

    if projectionAlong >= (exitDistanceAlong + hardPastDistance) then
        return true
    end

    local exitRouteIndex = maneuver.exitRouteIndex or maneuver.routeIndex
    if projection.segmentIndex ~= nil and exitRouteIndex ~= nil and projection.segmentIndex >= exitRouteIndex + navCfg("passedSegmentBuffer", 2) then
        if projectionAlong >= (exitDistanceAlong - passedMargin) then
            return true
        end
    end

    return false
end

function NavigationMode:hasReachedManeuverPoint(maneuver, projection, vehicleX, vehicleZ)
    if maneuver == nil or maneuver.point == nil then
        return false
    end

    local x, z = vehicleX, vehicleZ
    if x == nil or z == nil then
        x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    end

    local projectionAlong = self:getProjectionDistanceAlong(projection)
    local nodeReachDistance = navCfg("maneuverNodeReachDistance", 14)
    local nodeCaptureDistance = navCfg("maneuverNodeCaptureDistance", 24)
    local nodeReleaseDistance = navCfg("maneuverNodeReleaseDistance", 7)
    local nodeRouteMargin = navCfg("maneuverNodeRouteMargin", 0.8)
    local pointDistance = MathUtil.vector2Length(x - maneuver.point.x, z - maneuver.point.z)

    if self.lockedTurnPointId == maneuver.pointId and self.lockedTurnType == maneuver.turnType then
        if self.lockedTurnMinPointDistance == nil or pointDistance < self.lockedTurnMinPointDistance then
            self.lockedTurnMinPointDistance = pointDistance
        end
    end

    if pointDistance <= nodeReachDistance then
        return true
    end

    if self.lockedTurnPointId == maneuver.pointId and self.lockedTurnType == maneuver.turnType and self.lockedTurnMinPointDistance ~= nil then
        if self.lockedTurnMinPointDistance <= nodeCaptureDistance and pointDistance >= (self.lockedTurnMinPointDistance + nodeReleaseDistance) then
            return true
        end
    end

    if projection ~= nil and maneuver.distanceAlongRoute ~= nil and projectionAlong >= (maneuver.distanceAlongRoute + nodeRouteMargin) then
        if pointDistance <= navCfg("maneuverNodeProjectionConfirmDistance", 20) then
            return true
        end
    end

    return false
end

function NavigationMode:getApproachRouteIndex(projection)
    if self.route == nil or self.routeCumulative == nil or projection == nil then
        return nil
    end

    local along = self:getProjectionDistanceAlong(projection)
    local startIndex = math.max(1, projection.segmentIndex or 1)
    for index = startIndex + 1, #self.route do
        local distanceAlong = self.routeCumulative[index] or 0
        if distanceAlong > (along + navCfg("approachPointAheadMargin", 0.15)) then
            return index
        end
    end

    return #self.route
end

function NavigationMode:getContinuousDistanceToRouteIndex(routeIndex, vehicleX, vehicleZ, projection)
    if self.route == nil or self.routeCumulative == nil or routeIndex == nil then
        return nil
    end

    local x, z = vehicleX, vehicleZ
    if x == nil or z == nil then
        x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    end

    routeIndex = math.max(1, math.min(routeIndex, #self.route))
    local projectionAlong = self:getProjectionDistanceAlong(projection)
    local approachIndex = self:getApproachRouteIndex(projection) or routeIndex
    approachIndex = math.max(1, math.min(approachIndex, routeIndex))

    local firstPoint = self.route[approachIndex]
    if firstPoint == nil then
        return nil
    end

    if approachIndex >= routeIndex then
        local targetPoint = self.route[routeIndex]
        return MathUtil.vector2Length(x - targetPoint.x, z - targetPoint.z)
    end

    local firstLeg = MathUtil.vector2Length(x - firstPoint.x, z - firstPoint.z)
    local remaining = (self.routeCumulative[routeIndex] or 0) - (self.routeCumulative[approachIndex] or 0)
    return math.max(0, firstLeg + math.max(0, remaining))
end

function NavigationMode:getActiveManeuverDisplayDistance(maneuver, projection, vehicleX, vehicleZ)
    if maneuver == nil or maneuver.point == nil or projection == nil then
        return nil
    end

    if self:hasReachedManeuverPoint(maneuver, projection, vehicleX, vehicleZ) then
        return 0
    end

    local continuousDistance = self:getContinuousDistanceToRouteIndex(maneuver.routeIndex, vehicleX, vehicleZ, projection)
    if continuousDistance == nil then
        return nil
    end

    local x, z = vehicleX, vehicleZ
    if x == nil or z == nil then
        x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    end

    local directPointDistance = MathUtil.vector2Length(x - maneuver.point.x, z - maneuver.point.z)
    local guardRatio = navCfg("maneuverDistanceDirectGuardRatio", 0.35)
    return math.max(continuousDistance, directPointDistance * guardRatio)
end

function NavigationMode:smoothDistance(rawDistance, turnType, previousTurnType, isFarFromRoute, maneuverKey)
    if rawDistance == nil then
        self.activeDisplayManeuverKey = nil
        self.activeDisplayRawMin = nil
        return nil
    end

    local sameManeuver = maneuverKey ~= nil and maneuverKey == self.activeDisplayManeuverKey and previousTurnType == turnType and not isFarFromRoute
    if self.displayDistanceToTurn == nil or self.lastRawDistanceToTurn == nil or not sameManeuver then
        self.activeDisplayManeuverKey = maneuverKey
        self.activeDisplayRawMin = rawDistance
        return rawDistance
    end

    if self.activeDisplayRawMin == nil then
        self.activeDisplayRawMin = rawDistance
    else
        self.activeDisplayRawMin = math.min(self.activeDisplayRawMin, rawDistance)
    end

    local effectiveRaw = self.activeDisplayRawMin
    local previousDisplay = self.displayDistanceToTurn
    local dtMs = math.max(self.lastAnalysisDt or self.analysisInterval or 120, 60)
    local speedKmh = self:getVehicleSpeedKmh()
    local maxDrop = math.max(navCfg("distanceMinDropPerTick", 1.5), (speedKmh / 3.6) * (dtMs / 1000) * navCfg("distanceDropFactor", 1.7) + navCfg("distanceDropBuffer", 0.8))
    local maxRise = navCfg("distanceSameManeuverMaxRisePerTick", 0.05)

    if effectiveRaw <= previousDisplay then
        return math.max(effectiveRaw, previousDisplay - maxDrop)
    end

    return math.min(effectiveRaw, previousDisplay + maxRise)
end

function NavigationMode:getVehicleDirection()
    local node = self.vehicle.components[1].node
    local dx, _, dz = localDirectionToWorld(node, 0, 0, 1)
    return self:normalize2(dx, dz)
end

function NavigationMode:getHeadingDelta(route, projection)
    if route == nil or projection == nil then
        return nil
    end

    local routeDirX, routeDirZ = self:getRouteDirectionAtProjection(route, projection, navCfg("routeHeadingLookahead", 30))
    local vehicleDirX, vehicleDirZ = self:getVehicleDirection()
    if routeDirX == nil or vehicleDirX == nil then
        return nil
    end

    local dot = routeDirX * vehicleDirX + routeDirZ * vehicleDirZ
    dot = math.max(-1, math.min(1, dot))
    return math.deg(math.acos(dot))
end

function NavigationMode:getDirectionText(turnType)
    turnType = turnType or self.turnType

    if turnType == "left" then
        return g_i18n:getText("AD_NAV_LEFT")
    elseif turnType == "right" then
        return g_i18n:getText("AD_NAV_RIGHT")
    elseif turnType == "sharpLeft" then
        return g_i18n:getText("AD_NAV_SHARP_LEFT")
    elseif turnType == "sharpRight" then
        return g_i18n:getText("AD_NAV_SHARP_RIGHT")
    elseif turnType == "arrived" then
        return g_i18n:getText("AD_NAV_ARRIVED")
    elseif turnType == "noRoute" then
        return g_i18n:getText("AD_NAV_NO_ROUTE")
    elseif turnType == "uturn" then
        return g_i18n:getText("AD_NAV_UTURN")
    end

    return ""
end

function NavigationMode:getBarReferenceDistance(recommendedSpeed, turnType)
    if turnType == "straight" then
        return nil
    end
    if recommendedSpeed == nil then
        recommendedSpeed = 24
    end

    if turnType == "sharpLeft" or turnType == "sharpRight" then
        return navCfg("barRefSharp", 55)
    end

    if recommendedSpeed <= 12 then
        return navCfg("barRefSlow", 50)
    elseif recommendedSpeed <= 18 then
        return navCfg("barRefMedium", 65)
    elseif recommendedSpeed <= 26 then
        return navCfg("barRefFast", 85)
    elseif recommendedSpeed <= 34 then
        return navCfg("barRefVeryFast", 105)
    end

    return navCfg("barRefMax", 120)
end

function NavigationMode:getDistanceBarText(distance, recommendedSpeed, turnType)
    if distance == nil or turnType == "uturn" or turnType == "arrived" or turnType == "noRoute" or turnType == "straight" then
        return ""
    end

    local refDistance = self:getBarReferenceDistance(recommendedSpeed, turnType)
    if refDistance == nil or refDistance <= 0 then
        return ""
    end

    local slots = 8
    local clamped = math.max(0, math.min(refDistance, distance))
    local progress = 1 - (clamped / refDistance)
    local filled = math.max(1, math.ceil(progress * slots))
    if distance <= 4 then
        filled = slots
    end

    return string.rep("#", filled) .. string.rep("-", slots - filled)
end

function NavigationMode:getDistanceText(distance)
    if self.turnType == "uturn" then
        return ""
    end

    local barDistance = self.barDistanceToTurn
    if self.distanceBarText ~= nil and self.distanceBarText ~= "" and barDistance ~= nil then
        local roundedBarDistance = math.max(0, math.floor(barDistance + 0.5))
        return string.format("%s  %d m", self.distanceBarText, roundedBarDistance)
    end

    distance = distance or self.directDistanceToTurn or self.displayDistanceToTurn or self.distanceToTurn
    if distance == nil then
        distance = self.totalDistance
    end
    if distance == nil or self.turnType == "straight" then
        return ""
    end

    local roundedDistance = math.max(0, math.floor(distance + 0.5))
    return string.format("%d m", roundedDistance)
end

function NavigationMode:getDisplayInfo()
    if not self.vehicle.ad.stateModule:isNavigationActive() then
        return nil
    end

    local targetText = self.targetName
    if targetText ~= nil and targetText ~= "" then
        targetText = string.format("Navi: %s", targetText)
    else
        targetText = "Navi"
    end

    local nextText = ""

    return {
        targetText = targetText,
        directionText = self:getDirectionText(),
        distanceText = self:getDistanceText(),
        nextText = nextText
    }
end

function NavigationMode:getPreviewData()
    if not self.vehicle.ad.stateModule:isNavigationActive() then
        return nil
    end

    if self.route == nil or self.routeProjection == nil or #self.route < 2 then
        return nil
    end

    local maxDistance = 180
    if self.lockedTurnDistanceAlong ~= nil then
        maxDistance = math.min(320, math.max(120, self.lockedTurnDistanceAlong - self:getProjectionDistanceAlong(self.routeProjection) + 50))
    elseif self.distanceToTurn ~= nil then
        maxDistance = math.min(260, math.max(120, self.distanceToTurn + 40))
    end

    local endIndex = self.routeProjection.segmentIndex
    local segmentCount = #self.route - 1
    while endIndex < segmentCount do
        local distanceAtEnd = (self.routeCumulative[endIndex + 1] or 0) - self:getProjectionDistanceAlong(self.routeProjection)
        if distanceAtEnd >= maxDistance then
            break
        end
        endIndex = endIndex + 1
    end

    return {
        route = self.route,
        projection = self.routeProjection,
        startIndex = self.routeProjection.segmentIndex,
        endIndex = math.max(self.routeProjection.segmentIndex, endIndex)
    }
end

function NavigationMode:getVehicleSpeedKmh()
    if self.vehicle == nil then
        return 0
    end

    if self.vehicle.getLastSpeed ~= nil then
        return math.max(0, self.vehicle:getLastSpeed() * 3600)
    end

    local speed = self.vehicle.lastSpeedReal or self.vehicle.lastSpeed or 0
    if speed < 1 then
        speed = speed * 3600
    end

    return math.max(0, speed)
end

function NavigationMode:updateProgressState(projection, dt, vehicleSpeed)
    if projection == nil then
        self.progressSampleTimer = 0
        self.lastProgressDistanceAlong = nil
        self.lastProgressDelta = 0
        self.reverseProgressTimer = 0
        self.forwardProgressTimer = 0
        return
    end

    if self.lastProgressDistanceAlong == nil then
        self.lastProgressDistanceAlong = self:getProjectionDistanceAlong(projection)
        self.lastProgressDelta = 0
    end

    self.progressSampleTimer = self.progressSampleTimer + dt
    if self.progressSampleTimer >= navCfg("progressSampleMs", 450) then
        self.lastProgressDelta = projection.distanceAlongRoute - self.lastProgressDistanceAlong
        self.lastProgressDistanceAlong = self:getProjectionDistanceAlong(projection)
        self.progressSampleTimer = 0
    end

    local moving = vehicleSpeed ~= nil and vehicleSpeed >= 6
    if moving and self.lastProgressDelta <= -1.5 then
        self.reverseProgressTimer = math.min(4000, self.reverseProgressTimer + dt)
        self.forwardProgressTimer = math.max(0, self.forwardProgressTimer - dt * 2)
    elseif moving and self.lastProgressDelta >= 1.0 then
        self.forwardProgressTimer = math.min(4000, self.forwardProgressTimer + dt)
        self.reverseProgressTimer = math.max(0, self.reverseProgressTimer - dt * 3)
    else
        self.reverseProgressTimer = math.max(0, self.reverseProgressTimer - dt * 2)
        self.forwardProgressTimer = math.max(0, self.forwardProgressTimer - dt)
    end
end

function NavigationMode:isVehicleWrongWay(headingDelta, projection, totalDistance, vehicleSpeed, inDecisionZone)
    if headingDelta == nil or projection == nil then
        return false
    end
    if totalDistance == nil or totalDistance <= 20 then
        return false
    end
    if vehicleSpeed == nil or vehicleSpeed < navCfg("wrongWayMinSpeedKmh", 9) then
        return false
    end
    if projection.distanceToRoute > (inDecisionZone and navCfg("wrongWayMaxRouteDistanceIntersection", 18) or navCfg("wrongWayMaxRouteDistance", 12)) then
        return false
    end
    if headingDelta < navCfg("wrongWayHeadingDeg", 150) then
        return false
    end
    if self.reverseProgressTimer < navCfg("wrongWayReverseProgressMs", 1100) then
        return false
    end

    return true
end

function NavigationMode:getRouteDirectionAtProjection(route, projection, lookaheadDistance)
    if route == nil or projection == nil then
        return nil
    end

    local segmentIndex = projection.segmentIndex or 1
    local pointA = route[segmentIndex]
    local pointB = route[segmentIndex + 1]
    if pointA == nil or pointB == nil then
        return nil
    end

    local startX = pointA.x + (pointB.x - pointA.x) * (projection.t or 0)
    local startZ = pointA.z + (pointB.z - pointA.z) * (projection.t or 0)
    local sumX = 0
    local sumZ = 0
    local covered = 0

    local firstSegX = pointB.x - startX
    local firstSegZ = pointB.z - startZ
    local firstLen = MathUtil.vector2Length(firstSegX, firstSegZ)
    if firstLen > 0.01 then
        local useLen = math.min(firstLen, lookaheadDistance)
        sumX = sumX + (firstSegX / firstLen) * useLen
        sumZ = sumZ + (firstSegZ / firstLen) * useLen
        covered = covered + useLen
    end

    local index = segmentIndex + 1
    while covered < lookaheadDistance and index < #route do
        local prevPoint = route[index]
        local nextPoint = route[index + 1]
        if prevPoint == nil or nextPoint == nil then
            break
        end
        local segX = nextPoint.x - prevPoint.x
        local segZ = nextPoint.z - prevPoint.z
        local segLen = MathUtil.vector2Length(segX, segZ)
        if segLen > 0.01 then
            local useLen = math.min(segLen, lookaheadDistance - covered)
            sumX = sumX + (segX / segLen) * useLen
            sumZ = sumZ + (segZ / segLen) * useLen
            covered = covered + useLen
        end
        index = index + 1
    end

    return self:normalize2(sumX, sumZ)
end

function NavigationMode:getSmoothedRouteDirection(route, routeIndex, forward, lookDistance)
    local sumX = 0
    local sumZ = 0
    local covered = 0

    if forward then
        local index = routeIndex
        while covered < lookDistance and index < #route do
            local pointA = route[index]
            local pointB = route[index + 1]
            if pointA == nil or pointB == nil then
                break
            end
            local segX = pointB.x - pointA.x
            local segZ = pointB.z - pointA.z
            local segLen = MathUtil.vector2Length(segX, segZ)
            if segLen > 0.01 then
                local useLen = math.min(segLen, lookDistance - covered)
                sumX = sumX + (segX / segLen) * useLen
                sumZ = sumZ + (segZ / segLen) * useLen
                covered = covered + useLen
            end
            index = index + 1
        end
    else
        local index = routeIndex - 1
        while covered < lookDistance and index >= 1 do
            local pointA = route[index]
            local pointB = route[index + 1]
            if pointA == nil or pointB == nil then
                break
            end
            local segX = pointB.x - pointA.x
            local segZ = pointB.z - pointA.z
            local segLen = MathUtil.vector2Length(segX, segZ)
            if segLen > 0.01 then
                local useLen = math.min(segLen, lookDistance - covered)
                sumX = sumX + (segX / segLen) * useLen
                sumZ = sumZ + (segZ / segLen) * useLen
                covered = covered + useLen
            end
            index = index - 1
        end
    end

    return self:normalize2(sumX, sumZ)
end

function NavigationMode:getSmoothedBranchDirection(point, nextPointId, lookDistance)
    if point == nil or nextPointId == nil then
        return nil
    end

    local nextPoint = ADGraphManager:getWayPointById(nextPointId)
    if nextPoint == nil then
        return nil
    end

    local sumX = 0
    local sumZ = 0
    local covered = 0
    local previousPoint = point
    local currentPoint = nextPoint
    local safety = 0

    while currentPoint ~= nil and covered < lookDistance and safety < 12 do
        local segX = currentPoint.x - previousPoint.x
        local segZ = currentPoint.z - previousPoint.z
        local segLen = MathUtil.vector2Length(segX, segZ)
        if segLen <= 0.01 then
            break
        end

        local useLen = math.min(segLen, lookDistance - covered)
        sumX = sumX + (segX / segLen) * useLen
        sumZ = sumZ + (segZ / segLen) * useLen
        covered = covered + useLen

        if covered >= lookDistance then
            break
        end

        local nextId = self:getBestContinuationPointId(previousPoint, currentPoint)
        if nextId == nil then
            break
        end

        previousPoint = currentPoint
        currentPoint = ADGraphManager:getWayPointById(nextId)
        safety = safety + 1
    end

    return self:normalize2(sumX, sumZ)
end

function NavigationMode:getBestContinuationPointId(previousPoint, currentPoint)
    if currentPoint == nil or currentPoint.out == nil then
        return nil
    end

    local currentDirX, currentDirZ = self:normalize2(currentPoint.x - previousPoint.x, currentPoint.z - previousPoint.z)
    if currentDirX == nil then
        return nil
    end

    local bestId = nil
    local bestAbsAngle = math.huge

    for _, outId in pairs(currentPoint.out) do
        if previousPoint == nil or outId ~= previousPoint.id then
            local nextPoint = ADGraphManager:getWayPointById(outId)
            if nextPoint ~= nil then
                local nextDirX, nextDirZ = self:normalize2(nextPoint.x - currentPoint.x, nextPoint.z - currentPoint.z)
                if nextDirX ~= nil then
                    local angle = math.abs(self:getSignedAngleDeg(currentDirX, currentDirZ, nextDirX, nextDirZ))
                    if angle < bestAbsAngle then
                        bestAbsAngle = angle
                        bestId = outId
                    end
                end
            end
        end
    end

    return bestId
end

function NavigationMode:getSignedAngleDeg(fromX, fromZ, toX, toZ)
    local nFromX, nFromZ = self:normalize2(fromX, fromZ)
    local nToX, nToZ = self:normalize2(toX, toZ)
    if nFromX == nil or nToX == nil then
        return 0
    end

    local dot = nFromX * nToX + nFromZ * nToZ
    dot = math.max(-1, math.min(1, dot))
    local angle = math.deg(math.acos(dot))
    local cross = (nFromX * nToZ) - (nFromZ * nToX)
    if cross < 0 then
        return -angle
    end
    return angle
end

function NavigationMode:normalize2(x, z)
    local length = MathUtil.vector2Length(x, z)
    if length <= 0.0001 then
        return nil
    end
    return x / length, z / length
end
