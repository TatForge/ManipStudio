import QtQuick
import QtQuick3D
import Studio

// X / Y / Z axes of a frame as red / green / blue lines.
Node {
    id: axes
    property real length: 0.1

    Repeater3D {
        model: [
            { dir: Qt.vector3d(1, 0, 0), color: Theme.axisX },
            { dir: Qt.vector3d(0, 1, 0), color: Theme.axisY },
            { dir: Qt.vector3d(0, 0, 1), color: Theme.axisZ }
        ]
        delegate: Model {
            required property var modelData
            geometry: LineGeometry { points: [Qt.vector3d(0, 0, 0), modelData.dir.times(axes.length)] }
            materials: PrincipledMaterial { lighting: PrincipledMaterial.NoLighting; baseColor: modelData.color }
        }
    }
}
