## rough it out in python before scary cpp
## using old api ? like heck
## adapt or give up ccd

maya_useNewAPI = True

def maya_useNewAPI():
    pass

import sys
import maya.api.OpenMaya as om
import maya.OpenMaya as omOld
#import maya.api.OpenMayaMPx as omMPx
import math
import maya.cmds as cmds

kPluginNodeName = "generalIk"
kPluginNodeId = om.MTypeId( 0xDAE1 )


class generalIk(om.MPxNode):
    #define everything
    id = om.MTypeId( 0xDAE1)

    def __init__(self):
        om.MPxNode.__init__(self)

    def compute(self, pPlug, pData):

        #only compute if output is in out array
        if(pPlug.parent() == generalIk.aOutRot):
            # descend into coordinated cycles
            # inputs
            solverDH = pData.inputValue(generalIk.aSolver)
            solver = solverDH.asInt()

            iterationsDH = pData.inputValue(generalIk.aMaxIter)
            maxIter = iterationsDH.asInt()

            toleranceDH = pData.inputValue(generalIk.aTolerance)
            tolerance = toleranceDH.asFloat()

            rootMat = om.MMatrix()
            rootMatDH = pData.inputValue(generalIk.aRootMat)
            rootMat = rootMatDH.asMatrix()

            # target
            targetMat = om.MMatrix()
            targetMatDH = pData.inputValue(generalIk.aTargetMat)
            targetMat = targetMatDH.asMatrix()

            endMat = om.MMatrix()
            endMatDH = pData.inputValue(generalIk.aEndMat)
            endMat = endMatDH.asMatrix()
            print "endMat at start is {}".format(endMat)

            # get everything in root space
            targetRSmat = targetMat.__mul__(rootMat.inverse())

            # get reference matrix array from tip to root
            # this lets us virtually rebuild the parent-child hierarchy
            inJntArrayDH = pData.inputArrayValue(generalIk.aJnts)
            inLength = inJntArrayDH.__len__()
            # DON'T FORGET our useful arrays are one entry shorter than we have
            chainArray = om.MMatrixArray()
            jntArray = om.MMatrixArray()

            jntWeights = om.MFloatArray()

            for i in range(inLength):
                inJntArrayDH.jumpToPhysicalElement(i)
                childCompDH = inJntArrayDH.inputValue()

                childMat = om.MMatrix()
                childMatDH = om.MDataHandle(childCompDH.child(generalIk.aJntMat))
                #childUpMatDH = om.MDataHandle(childCompDH.child(generalIk.aJntUpMat))
                # not needed atm
                childMat = childMatDH.asMatrix()
                print "joint{}mat is {}".format(i, childMat)

                parentMat = om.MMatrix()
                if i != 0:
                    inJntArrayDH.jumpToPhysicalElement(i - 1)
                    parentCompDH = inJntArrayDH.inputValue()
                    parentMatDH = om.MDataHandle(parentCompDH.child(generalIk.aJntMat))
                    parentMat = parentMatDH.asMatrix()
                else:
                    # only applies to last in loop (so first in joints)
                    parentMat = rootMat

                chainMat = childMat.__mul__(parentMat.inverse())
                chainArray.append(chainMat)
                #chainArray is every joint in the space of its parent
                #chainArray[i] = jntMat[i] x jntMat[i-1].inverse
                jntArray.append(childMat)
                #childArray is just the joint matrices

                childWeightDH = om.MDataHandle(childCompDH.child(generalIk.aJntWeight))
                childWeight = childWeightDH.asFloat()
                print "childWeight is {}".format(childWeight)

                jntWeights.append(childWeight)

            print ""
            #from here applies only to ccd algorithm

            gap = 1.0
            iter = 0
            while (iter < maxIter) and (gap > tolerance):
                print "ITERATION {}".format(iter)
                print ""

                for i in range(inLength):

                    print "computing joint {} of {}".format(i, inLength)

                    # welcome to the bone zone

                    # reconstruct hierarchy with matrices from previous iteration
                    # currently target is known in rootspace, and end isn't known at all
                    # backwards to get target and forwards to get end, both in active joint space
                    #
                    #                  +(target)
                    #               .  
                    #             .
                    #           O 
                    #         /   \       X(end)
                    #       /       \   /
                    #     /           O
                    #   /
                    #(root)      
                    # this works by getting vector from active joint to end, from active joint to target,
                    # then aligning one to the other. joints are assumed to have direct parent-child
                    # hierarchy, so all rotations are inherited rigidly

                    # ccd operates from tip to root - see reference for actual algorithm
                    jntMat = om.MMatrix()
                    jntMat = jntArray.__getitem__(inLength - (i+1))
                    print "jntMat is {}".format(jntMat)
                    chainMult = om.MMatrix()

                    #backwards
                    for b in range(inLength - i):
                        #multiply targetRootSpaceMat by inverse chainMats,
                        #starting from ROOT
                        print "chainArray[b] is {}".format(chainArray[b])

                        if b == 0:
                            # at first joint, only equals root matrix
                            chainMult = chainArray[b].inverse()

                        else:
                            chainMult.__imul__(chainArray[b].inverse())

                        print "chainMult at {} is {}".format(b, chainMult)
                        print "endMat is {}".format(endMat)
                        print "endMat x chainMult is {}".format(endMat.__mul__(chainMult))

                    #forwards
                    # only need end in joint space
                    endJSmat = endMat.__mul__(jntMat.inverse())
                    targetJSmat = targetRSmat * chainMult

                    # check gap is still important
                    gapVec = om.MVector(targetJSmat[12] - endJSmat[12],
                        targetJSmat[13] - endJSmat[13],
                        targetJSmat[14] - endJSmat[14])
                    gap = gapVec.length()
                    if gap > tolerance:

                        print "gap is {}".format(gap)
                        print "end JS is {}".format(endJSmat)
                        print "endMat v2 at {} is {}".format(i, endMat)
                        print "target JS is {}".format(targetJSmat)
                        targetJStrans = om.MTransformationMatrix(targetJSmat).translation(1)
                        print "targetJStrans at {} is {}".format(i, targetJStrans)

                        # we don't just want to aim each joint, we want to
                        # aim the end, by rotating each joint in turn

                        # first get the aim matrix from joint to end
                        endAimMat = om.MMatrix()
                        endAimMat = lookAt(jntMat, endJSmat)
                        print "endAimMat is {}".format(endAimMat)
                        #jntMat.__imul__(endAimMat)

                        # then from that, aim from end to target
                        targetAimMat = om.MMatrix()
                        targetAimMat = lookAt(jntMat, targetJSmat)

                        # is weighting this simple?
                        #print "jntWeight is {}".format(jntWeights.__getitem__(i))
                        targetAimMat.__imul__(jntWeights.__getitem__(i))
                        print "targetAimMat is {}".format(targetAimMat)

                        #constraints are going to be fun, but for now
                        jntMat.__imul__(targetAimMat)
                        endMat.__imul__(targetAimMat)

                        print "finalEndMat is {}".format(endMat)

                        #end of if block and aim procedure
                        print ""
                    else:
                        print ""
                        print "gap is within tolerance, ending"
                        break

                    jntArray.__setitem__(i, jntMat)
                    #end of single iteration along joint chain
                print ""

                iter = iter + 1

                #end of all iterations, computation has completed
            
            #convert jntArray of matrices to useful rotation values
            outArrayDH = pData.outputArrayValue(generalIk.aOutArray)

            targetRSTransA = om.MTransformationMatrix(targetRSmat).translation(4)
            print "target in RS world is {}".format(targetRSTransA)
            
            for i in range(0, inLength):
                outArrayDH.jumpToPhysicalElement(i)
                outCompDH = outArrayDH.outputValue()

                outRotDH = outCompDH.child(generalIk.aOutRot)
                outRxDH = outRotDH.child(generalIk.aOutRx)
                outRyDH = outRotDH.child(generalIk.aOutRy)
                outRzDH = outRotDH.child(generalIk.aOutRz)

                outRotVals = om.MTransformationMatrix(jntArray[i]).rotation()
                # unitConversions bring SHAME on family
                xAngle = om.MAngle(outRotVals[0])
                yAngle = om.MAngle(outRotVals[1])
                zAngle = om.MAngle(outRotVals[2])
                outRxDH.setMAngle( xAngle )
                outRyDH.setMAngle( yAngle )
                outRzDH.setMAngle( zAngle )

            outArrayDH.setAllClean()

            pData.setClean(pPlug)

        # else:
        #     return om.kUnknownParameter
def lookAt(base, target, up = [0, 1, 0]):
    #axes one by one, code shamelessly copied from somewhere
    #convert to quat someday?

    # x is vector between base and target
    x = om.MVector(target[12]-base[12], target[13]-base[13], target[14]-base[14])

    x.normalize()

    z = x ^ om.MVector(-up[0], -up[1], up[2])
    z.normalize()
    y = x ^ z
    y.normalize()

    aim = om.MMatrix()
    aim.__setitem__(0, x.x)
    aim.__setitem__(1, x.y)
    aim.__setitem__(2, x.z)
    aim.__setitem__(3, 0)
    aim.__setitem__(4, y.x)
    aim.__setitem__(5, y.y)
    aim.__setitem__(6, y.z)
    aim.__setitem__(7, 0)
    aim.__setitem__(8, z.x)
    aim.__setitem__(9, z.y)
    aim.__setitem__(10, z.z)
    aim.__setitem__(11, 0)
    aim.__setitem__(12, 0)
    aim.__setitem__(13, 0)
    aim.__setitem__(14, 0)
    aim.__setitem__(15, 1)

    return aim
    #if there's a way to make a MMatrix from a list i'd really love to hear about it


def nodeInitializer():
    # create attributes

    # pick your pointy poison
    solverAttrFn = om.MFnEnumAttribute()
    generalIk.aSolver = solverAttrFn.create("solver", "sol", 0)
    solverAttrFn.addField("CCD", 0)
    solverAttrFn.addField("FABRIK (not yet implemented)", 1)
    solverAttrFn.storable = True
    solverAttrFn.keyable = True
    solverAttrFn.readable = False
    solverAttrFn.writable = True
    om.MPxNode.addAttribute(generalIk.aSolver)

    # don't take too much now
    iterAttrFn = om.MFnNumericAttribute()
    generalIk.aMaxIter = iterAttrFn.create("maxIterations", "mi",
        om.MFnNumericData.kLong, 30)
    iterAttrFn.storable = True
    iterAttrFn.keyable = True
    iterAttrFn.readable = False
    iterAttrFn.writable = True
    iterAttrFn.setMin(0)
    om.MPxNode.addAttribute(generalIk.aMaxIter)

    # how far will you go for perfection
    toleranceAttrFn = om.MFnNumericAttribute()
    generalIk.aTolerance = toleranceAttrFn.create("tolerance", "tol",
        om.MFnNumericData.kDouble, 0.01)
    toleranceAttrFn.storable = True
    toleranceAttrFn.keyable = True
    toleranceAttrFn.readable = False
    toleranceAttrFn.writable = True
    toleranceAttrFn.setMin(0)
    om.MPxNode.addAttribute(generalIk.aTolerance)

    # everyone needs a place to call home
    rootMatAttrFn = om.MFnMatrixAttribute()
    generalIk.aRootMat = rootMatAttrFn.create("rootMatrix", "rootMat",
        1) #specifies should be matrix of doubles
    rootMatAttrFn.storable = True
    rootMatAttrFn.keyable = True
    rootMatAttrFn.readable = True
    rootMatAttrFn.writable = True
    rootMatAttrFn.cached = True
    om.MPxNode.addAttribute(generalIk.aRootMat)

    # what are your goals in life
    targetMatAttrFn = om.MFnMatrixAttribute()
    generalIk.aTargetMat = targetMatAttrFn.create("targetMatrix",
    "targetMat", 1)
    targetMatAttrFn.storable = True
    targetMatAttrFn.readable = False
    targetMatAttrFn.writable = True
    targetMatAttrFn.cached = True
    om.MPxNode.addAttribute(generalIk.aTargetMat)
    #
    # # how badly do you want them
    # targetWeightAttrFn = om.MFnNumericAttribute()
    # generalIk.aTargetWeight = targetWeightAttrFn.create("targetWeight",
    #     "tWeight", om.MFnNumericData.kDouble, 1.0)
    # targetWeightAttrFn.storable = True
    # targetWeightAttrFn.keyable = True
    # targetWeightAttrFn.readable = False
    # targetWeightAttrFn.writable = True
    # targetWeightAttrFn.setMin(0)
    # targetWeightAttrFn.setMax(1)
    # om.MPxNode.addAttribute(generalIk.aTargetWeight)
    # not doing multiple targets anymore

    endMatAttrFn = om.MFnMatrixAttribute()
    generalIk.aEndMat = endMatAttrFn.create("endMatrix", "endMat", 1)
    endMatAttrFn.storable = True
    endMatAttrFn.readable = False
    endMatAttrFn.writable = True
    endMatAttrFn.cached = True
    om.MPxNode.addAttribute(generalIk.aEndMat)

    # # keep your focus
    # targetsArrayAttrFn = om.MFnCompoundAttribute()
    # generalIk.aTargets = targetsArrayAttrFn.create("targetsArray", "tArray")
    # targetsArrayAttrFn.array = True
    # targetsArrayAttrFn.usesArrayDataBuilder = True
    # targetsArrayAttrFn.addChild(generalIk.aTargetMat)
    # targetsArrayAttrFn.addChild(generalIk.aTargetWeight)
    # om.MPxNode.addAttribute(generalIk.aTargets)
    # multiple targets are not necessary, if you disagree you're wrong


    # once i built a tower
    jntMatAttrFn = om.MFnMatrixAttribute()
    generalIk.aJntMat = jntMatAttrFn.create("jointMatrix",
    "jntMat", 1)
    jntMatAttrFn.storable = True
    jntMatAttrFn.keyable = True
    jntMatAttrFn.readable = True
    jntMatAttrFn.writable = True
    jntMatAttrFn.cached = True
    #om.MPxNode.addAttribute(generalIk.aJntMat)

    # eye on the sky
    jntUpMatAttrFn = om.MFnMatrixAttribute()
    generalIk.aJntUpMat = jntUpMatAttrFn.create("jointUpMatrix",
    "jntUpMat", 1)
    jntUpMatAttrFn.storable = True
    jntUpMatAttrFn.keyable = True
    jntUpMatAttrFn.readable = True
    jntUpMatAttrFn.writable = True
    jntUpMatAttrFn.cached = True
    #om.MPxNode.addAttribute(generalIk.aJntUpMat)

    # who is the heftiest boi
    jntWeightAttrFn = om.MFnNumericAttribute()
    generalIk.aJntWeight = jntWeightAttrFn.create("jointWeight", "jntWeight",
        om.MFnNumericData.kFloat, 1)
    jntWeightAttrFn.storable = True
    jntWeightAttrFn.keyable = True
    jntWeightAttrFn.readable = True
    jntWeightAttrFn.writable = True
    jntWeightAttrFn.setMin(0)
    #om.MPxNode.addAttribute(generalIk.aJntWeight)

    # know your limits
    limRxAttrFn = om.MFnNumericAttribute()
    generalIk.aLimRx = limRxAttrFn.create("limitRotateX", "limRx",
        om.MFnNumericData.kBoolean, False)
    limRxAttrFn.storable = True
    limRxAttrFn.keyable = True
    om.MPxNode.addAttribute(generalIk.aLimRx)

    # like really know them
    rxMaxAttrFn = om.MFnNumericAttribute()
    generalIk.aRxMax = rxMaxAttrFn.create("maxRotateX", "maxRx",
        om.MFnNumericData.kFloat, 0)
    limRxAttrFn.storable = True
    limRxAttrFn.keyable = True
    om.MPxNode.addAttribute(generalIk.aRxMax)

    # how low can you go
    rxMinAttrFn = om.MFnNumericAttribute()
    generalIk.aRxMin = rxMinAttrFn.create("minRotateX", "minRx",
        om.MFnNumericData.kFloat, 0)
    limRxAttrFn.storable = True
    limRxAttrFn.keyable = True
    om.MPxNode.addAttribute(generalIk.aRxMin)

    ## there is more to be done here

    # you will never break the chain
    jntArrayAttrFn = om.MFnCompoundAttribute()
    generalIk.aJnts = jntArrayAttrFn.create("joints", "joints")
    jntArrayAttrFn.array = True
    jntArrayAttrFn.usesArrayDataBuilder = True
    jntArrayAttrFn.addChild(generalIk.aJntMat)
    jntArrayAttrFn.addChild(generalIk.aJntUpMat)
    jntArrayAttrFn.addChild(generalIk.aJntWeight)
    # add limits later
    om.MPxNode.addAttribute(generalIk.aJnts)



    aJntLimRy = None
    aJntRyMax = None
    aJntRyMin = None

    aJntLimRz = None
    aJntRzMax = None
    aJntRzMin = None


    aJntLimTx = None
    aJntTxMax = None
    aJntTxMin = None

    aJntLimTy = None
    aJntTyMax = None
    aJntTyMin = None

    aJntLimTz = None
    aJntTzMax = None
    aJntTzMin = None

    # fruits of labour
    outRxAttrFn = om.MFnUnitAttribute()
    generalIk.aOutRx = outRxAttrFn.create("outputRotateX", "outRx", 1, 0.0)
    outRxAttrFn.writable = False
    outRxAttrFn.keyable = False
    #om.MPxNode.addAttribute(generalIk.aOutRx)


    outRyAttrFn = om.MFnUnitAttribute()
    generalIk.aOutRy = outRyAttrFn.create("outputRotateY", "outRy", 1, 0.0)
    outRyAttrFn.writable = False
    outRyAttrFn.keyable = False
    #om.MPxNode.addAttribute(generalIk.aOutRy)

    outRzAttrFn = om.MFnUnitAttribute()
    generalIk.aOutRz = outRzAttrFn.create("outputRotateZ", "outRz", 1, 0.0)
    outRzAttrFn.writable = False
    outRzAttrFn.keyable = False
    #om.MPxNode.addAttribute(generalIk.aOutRz)

    outRotAttrFn = om.MFnCompoundAttribute()
    # generalIk.aOutRot = outRotAttrFn.create("outputRotate", "outRot",
    #     om.MFnNumericData.k3Double)
    generalIk.aOutRot = outRotAttrFn.create("outputRotate", "outRot")
    outRotAttrFn.storable = False
    outRotAttrFn.writable = False
    outRotAttrFn.keyable = False
    outRotAttrFn.addChild(generalIk.aOutRx)
    outRotAttrFn.addChild(generalIk.aOutRy)
    outRotAttrFn.addChild(generalIk.aOutRz)
    om.MPxNode.addAttribute(generalIk.aOutRot)


    # add smooth jazz
    outTxAttrFn = om.MFnNumericAttribute()
    aOutTx = outTxAttrFn.create("outputTranslateX", "outTx",
        om.MFnNumericData.kDouble, 0)
    outTxAttrFn.writable = False
    outTxAttrFn.keyable = False

    outTyAttrFn = om.MFnNumericAttribute()
    aOutTy = outTyAttrFn.create("outputTranslateY", "outTy",
        om.MFnNumericData.kDouble, 0)
    outTyAttrFn.writable = False
    outTyAttrFn.keyable = False

    outTzAttrFn = om.MFnNumericAttribute()
    aOutTz = outTzAttrFn.create("outputTranslateZ", "outTz",
        om.MFnNumericData.kDouble, 0)
    outTzAttrFn.writable = False
    outTzAttrFn.keyable = False

    outTransAttrFn = om.MFnCompoundAttribute()
    generalIk.aOutTrans = outTransAttrFn.create("outputTranslate", "outTrans")
    outTransAttrFn.storable = False
    outTransAttrFn.writable = False
    outTransAttrFn.keyable = False
    outTransAttrFn.addChild(aOutTx)
    outTransAttrFn.addChild(aOutTy)
    outTransAttrFn.addChild(aOutTz)
    om.MPxNode.addAttribute(generalIk.aOutTrans)


    # all that the sun touches
    outArrayAttrFn = om.MFnCompoundAttribute()
    generalIk.aOutArray = outArrayAttrFn.create("outputArray", "out")
    outArrayAttrFn.array = True
    outArrayAttrFn.usesArrayDataBuilder = True
    outArrayAttrFn.storable = False
    outArrayAttrFn.writable = False
    outArrayAttrFn.keyable = False
    outArrayAttrFn.addChild(generalIk.aOutRot)
    outArrayAttrFn.addChild(generalIk.aOutTrans)
    om.MPxNode.addAttribute(generalIk.aOutArray)


    #tech fixes
    #rebuildAttrFn = om.MFnGenericAttribute()


    # everyone's counting on you
    generalIk.attributeAffects(generalIk.aTargetMat, generalIk.aOutArray)
    generalIk.attributeAffects(generalIk.aMaxIter, generalIk.aOutArray)
    generalIk.attributeAffects(generalIk.aTolerance, generalIk.aOutArray)


    # following are reference chain - trigger rebuild only when these change
    generalIk.attributeAffects(generalIk.aRootMat, generalIk.aOutArray)
    generalIk.attributeAffects(generalIk.aEndMat, generalIk.aOutArray)
    generalIk.attributeAffects(generalIk.aJnts, generalIk.aOutArray)
    
#     # TRY THIS OUT LATER:
#     refMatArrayFn = om.MFnMatrixAttribute()
#     generalIk.aRefArray = refMatArrayFn.create("refMatArray")
#     refMatArrayFn.array = True
#     refMatArrayFn.internal = True
#     refMatArrayFn.cached = True
#     refMatArrayFn.storable = True
#     # do we need arrayDataBuilder?
#     om.MPxNode.addAttribute(generalIk.aRefArray)
# this would be constructed of the joints' relative matrices, and then store the outputs
# of the node across iterations - basically its memory
# if the reference skeleton changes RELATIVE TO THE ROOT, this would need to be recalculated
# a battle for another day

def nodeCreator():
        # creates node, returns to maya as pointer
        return generalIk()

def initializePlugin(mobject):
    mplugin = om.MFnPlugin(mobject)
    try:
        mplugin.registerNode( kPluginNodeName, kPluginNodeId,
        nodeCreator, nodeInitializer)
    except:
        sys.stderr.write("Failed to register node:" + kPluginNodeName)
        raise

def uninitializePlugin( mobject ):
    mPlugin = om.MFnPlugin(mobject)
    try:
        mPlugin.deregisterNode(kPluginNodeId)
    except:
        sys.stderr.write("failed to unregister node, you're stuck with generalIk forever lol")
        raise

# roadmap:
# get it working
# get it working with constraints
# find way to cache matrix chain unless reference chain rebuilds
# get different solvers working - maybe fabrik, but i want to try the quat splice
# rebuild in c++ if it really needs it?
# make cmd to attach joints automatically
