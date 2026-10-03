import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import Studio

// 3D view of the arm: a display model built from the joint frames (always
// true to the kinematics), optionally the coarse STL parts the collision
// check uses, plus the floor, the obstacle and the TCP path.
// The scene is in millimetres (Quick3D defaults suit that scale); the robot
// node converts from the robot's metres and Z-up frame.
Rectangle {
    id: root
    color: Theme.window
    border.color: Theme.panelBorder
    radius: Theme.radius
    clip: true

    readonly property color linkColor: "#c9ced6"
    readonly property color jointColor: "#f08a24"
    readonly property color baseColor: "#4a505a"
    readonly property color flangeColor: "#2f343c"
    property bool showCollision: false
    // orbit centre: middle of the reach, robot (0.3, 0.15, 0.25) m in scene millimetres
    readonly property vector3d homeTarget: Qt.vector3d(300, 250, -150)
    readonly property real homeDistance: 2200

    function roleColor(role) {
        switch (role) {
        case "base": return baseColor
        case "joint": return jointColor
        case "flange": return flangeColor
        default: return linkColor
        }
    }

    // floor grid: segments every `step` metres within +-`half`
    function gridPoints(half, step) {
        const pts = []
        for (let v = -half; v <= half + 1e-9; v += step) {
            pts.push(Qt.vector3d(v, -half, 0), Qt.vector3d(v, half, 0))
            pts.push(Qt.vector3d(-half, v, 0), Qt.vector3d(half, v, 0))
        }
        return pts
    }

    function setView(pitch, yaw) {
        orbit.eulerRotation = Qt.vector3d(pitch, yaw, 0)
        orbit.position = homeTarget
        camera.z = homeDistance
    }

    View3D {
        id: view
        anchors.fill: parent
        anchors.margins: 1

        environment: SceneEnvironment {
            clearColor: Theme.window
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: orbit
            position: root.homeTarget
            eulerRotation: Qt.vector3d(-25, 35, 0)
            PerspectiveCamera {
                id: camera
                z: root.homeDistance
                fieldOfView: 40
                clipNear: 10
                clipFar: 30000
            }
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-55, 25, 0)
            brightness: 1.0
            castsShadow: true
            shadowFactor: 35
            shadowMapQuality: Light.ShadowMapQualityHigh
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(-20, -150, 0)
            brightness: 0.45
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(80, 0, 0)
            brightness: 0.15
        }

        Node {
            id: robotFrame
            // robot: metres, Z up  ->  scene: millimetres, Y up
            eulerRotation.x: -90
            scale: Qt.vector3d(1000, 1000, 1000)

            Model {
                source: "#Rectangle"
                z: robot.floorZ - 0.001
                scale: Qt.vector3d(0.05, 0.05, 1)      // 5 x 5 m
                receivesShadows: true
                materials: PrincipledMaterial { baseColor: "#1a1e24"; roughness: 0.95 }
            }
            Model {
                z: robot.floorZ
                geometry: LineGeometry { segments: true; points: root.gridPoints(2.5, 0.25) }
                materials: PrincipledMaterial { lighting: PrincipledMaterial.NoLighting; baseColor: "#2c323b" }
            }

            AxisLines { length: 0.25 }

            // display model: one delegate per part, the count only changes with the geometry
            Repeater3D {
                model: robot.visualPartCount
                delegate: Model {
                    required property int index
                    readonly property var part: robot.visualParts[index]
                    visible: part !== undefined
                    source: part && part.shape === "sphere" ? "#Sphere" : "#Cylinder"
                    position: part ? part.position : Qt.vector3d(0, 0, 0)
                    rotation: part ? part.rotation : Qt.quaternion(1, 0, 0, 0)
                    scale: part ? part.scale : Qt.vector3d(0, 0, 0)
                    castsShadows: true
                    receivesShadows: true
                    materials: PrincipledMaterial {
                        baseColor: root.roleColor(part ? part.role : "")
                        // low metalness: without an environment map metal renders almost black
                        metalness: part && part.role === "link" ? 0.2 : 0.1
                        roughness: part && part.role === "link" ? 0.35 : 0.45
                    }
                }
            }

            // the STL parts the collision check works with; the obstacle exists only here
            Repeater3D {
                model: robot.meshParts
                delegate: Model {
                    required property var modelData
                    required property int index
                    readonly property var pose: robot.meshPoses[index]
                    visible: modelData.obstacle || root.showCollision
                    position: pose ? pose.position : Qt.vector3d(0, 0, 0)
                    rotation: pose ? pose.rotation : Qt.quaternion(1, 0, 0, 0)
                    geometry: StlGeometry { source: modelData.source }
                    castsShadows: modelData.obstacle
                    receivesShadows: true
                    opacity: modelData.obstacle ? 0.75 : 0.45
                    materials: PrincipledMaterial {
                        baseColor: modelData.obstacle ? "#9b3b3b" : "#4fd1c5"
                        roughness: 0.5
                    }
                }
            }

            Model {
                visible: robot.tcpPath.length > 1
                geometry: LineGeometry { points: robot.tcpPath }
                materials: PrincipledMaterial { lighting: PrincipledMaterial.NoLighting; baseColor: Theme.warn }
            }

            AxisLines {
                length: 0.1
                position: robot.toolPose.position !== undefined ? robot.toolPose.position : Qt.vector3d(0, 0, 0)
                rotation: robot.toolPose.rotation !== undefined ? robot.toolPose.rotation : Qt.quaternion(1, 0, 0, 0)
            }
        }
    }

    OrbitCameraController {
        anchors.fill: view
        origin: orbit
        camera: camera
    }

    RowLayout {
        anchors { right: parent.right; top: parent.top; margins: Theme.gap }
        spacing: 4
        Chip {
            text: qsTr("Collision model")
            tint: "#4fd1c5"
            checked: root.showCollision
            onClicked: root.showCollision = !root.showCollision
            Layout.rightMargin: 8
        }
        Repeater {
            model: [
                { label: qsTr("Iso"), pitch: -25, yaw: 35 },
                { label: qsTr("Front"), pitch: 0, yaw: 0 },
                { label: qsTr("Side"), pitch: 0, yaw: 90 },
                { label: qsTr("Top"), pitch: -90, yaw: 0 }
            ]
            delegate: Button {
                required property var modelData
                text: modelData.label
                flat: true
                font.pixelSize: 12
                onClicked: root.setView(modelData.pitch, modelData.yaw)
            }
        }
    }

    Text {
        text: qsTr("LMB: rotate · wheel: zoom · RMB / two fingers: pan")
        color: Theme.textDim
        font.pixelSize: 12
        anchors { left: parent.left; bottom: parent.bottom; margins: Theme.gap }
    }
}
