import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Settings editor: a draft copy of robot.settings, edited here and handed
// back with robot.applySettings(draft, save). Units as shown in the headers.
Drawer {
    id: drawer
    edge: Qt.RightEdge
    width: Math.min(620, parent ? parent.width * 0.9 : 620)
    height: parent ? parent.height : 600
    modal: true

    property var draft: ({})
    property int page: 0
    readonly property bool dirty: JSON.stringify(draft) !== JSON.stringify(robot.settings)

    function revert() { draft = JSON.parse(JSON.stringify(robot.settings)) }
    // edits a deep copy so that every binding on `draft` re-evaluates
    function edit(change) {
        const d = JSON.parse(JSON.stringify(draft))
        change(d)
        draft = d
    }
    function value(list, row, key) {
        return draft[list] && draft[list][row] !== undefined ? draft[list][row][key] : 0
    }

    onOpened: revert()
    Connections {
        target: robot
        // after Apply / Save / Reload the draft starts again from what the model now holds
        function onSettingsChanged() { drawer.revert() }
    }

    background: Rectangle { color: Theme.window; border.color: Theme.panelBorder }
    Overlay.modal: Rectangle { color: "#99000000" }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Theme.gap
        spacing: Theme.gap

        RowLayout {
            Layout.fillWidth: true
            Text { text: qsTr("Settings"); color: Theme.text; font.pixelSize: 18; font.bold: true }
            Text {
                visible: drawer.dirty
                text: qsTr("unsaved changes")
                color: Theme.warn
                Layout.leftMargin: Theme.gap
            }
            Item { Layout.fillWidth: true }
            Button { text: "✕"; flat: true; onClicked: drawer.close() }
        }

        RowLayout {
            spacing: 6
            Repeater {
                model: [qsTr("Kinematics"), qsTr("Limits"), qsTr("Dynamics")]
                delegate: Chip {
                    required property var modelData
                    required property int index
                    text: modelData
                    checked: drawer.page === index
                    onClicked: drawer.page = index
                }
            }
        }

        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentWidth: availableWidth
            clip: true

            StackLayout {
                width: parent.width
                currentIndex: drawer.page

                // ---------------------------------------------------- kinematics
                ColumnLayout {
                    spacing: 6
                    Text {
                        Layout.fillWidth: true
                        wrapMode: Text.Wrap
                        color: Theme.textDim
                        font.pixelSize: 12
                        text: qsTr("Modified Denavit-Hartenberg table: frame i is reached from frame i-1 by α and a of row i-1, then θ and d of row i. The radius sets the joint drum of the 3D and collision model.")
                    }
                    GridLayout {
                        columns: 5
                        columnSpacing: 8
                        rowSpacing: 2
                        Repeater {
                            model: [qsTr("Joint"), qsTr("a, m"), qsTr("α, °"), qsTr("d, m"), qsTr("Radius, m")]
                            delegate: Text { required property var modelData; text: modelData; color: Theme.textDim; font.pixelSize: 12 }
                        }
                        Repeater {
                            model: 6
                            delegate: Text {
                                required property int index
                                Layout.row: index + 1; Layout.column: 0
                                text: "q" + (index + 1); color: Theme.jointColors[index]; font.bold: true
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 1
                                value: drawer.value("dh", index, "a")
                                onEdited: v => drawer.edit(d => d.dh[index].a = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 2
                                value: drawer.value("dh", index, "alpha")
                                onEdited: v => drawer.edit(d => d.dh[index].alpha = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 3
                                value: drawer.value("dh", index, "d")
                                onEdited: v => drawer.edit(d => d.dh[index].d = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 4
                                value: drawer.draft.jointRadii ? drawer.draft.jointRadii[index] : 0
                                onEdited: v => drawer.edit(d => d.jointRadii[index] = v)
                            }
                        }
                    }
                }

                // ---------------------------------------------------- limits
                ColumnLayout {
                    spacing: 6
                    Text {
                        Layout.fillWidth: true
                        wrapMode: Text.Wrap
                        color: Theme.textDim
                        font.pixelSize: 12
                        text: qsTr("MoveJ keeps every joint within its speed and acceleration; MoveL and MoveC slow the whole move down instead of leaving the path.")
                    }
                    GridLayout {
                        columns: 6
                        columnSpacing: 8
                        rowSpacing: 2
                        Repeater {
                            model: [qsTr("Joint"), qsTr("min, °"), qsTr("max, °"), qsTr("Speed, °/s"), qsTr("Accel., °/s²"), qsTr("Moment, N·m")]
                            delegate: Text { required property var modelData; text: modelData; color: Theme.textDim; font.pixelSize: 12 }
                        }
                        Repeater {
                            model: 6
                            delegate: Text {
                                required property int index
                                Layout.row: index + 1; Layout.column: 0
                                text: "q" + (index + 1); color: Theme.jointColors[index]; font.bold: true
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 1
                                value: drawer.value("limits", index, "min")
                                onEdited: v => drawer.edit(d => d.limits[index].min = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 2
                                value: drawer.value("limits", index, "max")
                                onEdited: v => drawer.edit(d => d.limits[index].max = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 3
                                value: drawer.value("limits", index, "speed")
                                onEdited: v => drawer.edit(d => d.limits[index].speed = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 4
                                value: drawer.value("limits", index, "accel")
                                onEdited: v => drawer.edit(d => d.limits[index].accel = v)
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 5
                                value: drawer.draft.maxMoments ? drawer.draft.maxMoments[index] : 0
                                onEdited: v => drawer.edit(d => d.maxMoments[index] = v)
                            }
                        }
                    }
                    RowLayout {
                        Layout.topMargin: Theme.gap
                        Text { text: qsTr("Floor height, m"); color: Theme.textDim }
                        NumberField {
                            value: drawer.draft.floorZ !== undefined ? drawer.draft.floorZ : 0
                            onEdited: v => drawer.edit(d => d.floorZ = v)
                        }
                    }
                }

                // ---------------------------------------------------- dynamics
                ColumnLayout {
                    spacing: 6
                    Text { text: qsTr("Payload (point mass in the flange frame)"); color: Theme.text; font.bold: true }
                    RowLayout {
                        spacing: 8
                        Text { text: qsTr("Mass, kg"); color: Theme.textDim }
                        NumberField {
                            value: drawer.draft.payload ? drawer.draft.payload.mass : 0
                            onEdited: v => drawer.edit(d => d.payload.mass = v)
                        }
                        Text { text: qsTr("Centre of mass x, y, z, m"); color: Theme.textDim; Layout.leftMargin: Theme.gap }
                        Repeater {
                            model: 3
                            delegate: NumberField {
                                required property int index
                                value: drawer.draft.payload ? drawer.draft.payload.com[index] : 0
                                onEdited: v => drawer.edit(d => d.payload.com[index] = v)
                            }
                        }
                    }

                    Text { text: qsTr("Links: mass, kg and centre of mass in the joint frame, m"); color: Theme.text; font.bold: true; Layout.topMargin: Theme.gap }
                    GridLayout {
                        columns: 5
                        columnSpacing: 8
                        rowSpacing: 2
                        Repeater {
                            model: [qsTr("Link"), qsTr("Mass, kg"), "x", "y", "z"]
                            delegate: Text { required property var modelData; text: modelData; color: Theme.textDim; font.pixelSize: 12 }
                        }
                        Repeater {
                            model: 6
                            delegate: Text {
                                required property int index
                                Layout.row: index + 1; Layout.column: 0
                                text: String(index + 1); color: Theme.jointColors[index]; font.bold: true
                            }
                        }
                        Repeater {
                            model: 6
                            delegate: NumberField {
                                required property int index
                                Layout.row: index + 1; Layout.column: 1
                                value: drawer.value("links", index, "mass")
                                onEdited: v => drawer.edit(d => d.links[index].mass = v)
                            }
                        }
                        Repeater {
                            model: 18
                            delegate: NumberField {
                                required property int index
                                readonly property int link: Math.floor(index / 3)
                                readonly property int axis: index % 3
                                Layout.row: link + 1; Layout.column: 2 + axis
                                value: drawer.draft.links ? drawer.draft.links[link].com[axis] : 0
                                onEdited: v => drawer.edit(d => d.links[link].com[axis] = v)
                            }
                        }
                    }

                    Text { text: qsTr("Inertia about the joint frame origin, kg·m²"); color: Theme.text; font.bold: true; Layout.topMargin: Theme.gap }
                    GridLayout {
                        columns: 7
                        columnSpacing: 6
                        rowSpacing: 2
                        Repeater {
                            model: [qsTr("Link"), "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
                            delegate: Text { required property var modelData; text: modelData; color: Theme.textDim; font.pixelSize: 12 }
                        }
                        Repeater {
                            model: 6
                            delegate: Text {
                                required property int index
                                Layout.row: index + 1; Layout.column: 0
                                text: String(index + 1); color: Theme.jointColors[index]; font.bold: true
                            }
                        }
                        Repeater {
                            model: 36
                            delegate: NumberField {
                                required property int index
                                readonly property int link: Math.floor(index / 6)
                                readonly property int k: index % 6
                                implicitWidth: 64
                                Layout.row: link + 1; Layout.column: 1 + k
                                value: drawer.draft.links ? drawer.draft.links[link].inertia[k] : 0
                                onEdited: v => drawer.edit(d => d.links[link].inertia[k] = v)
                            }
                        }
                    }
                }
            }
        }

        Text {
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            color: Theme.textDim
            font.pixelSize: 12
            text: qsTr("Apply rebuilds the planner with these values; Save also writes them to %1.").arg(robot.dataDir + "/ManipulatorSettigns.ini")
        }

        RowLayout {
            Layout.fillWidth: true
            Button {
                text: qsTr("Revert")
                flat: true
                enabled: drawer.dirty
                onClicked: drawer.revert()
            }
            Button {
                text: qsTr("Reload file")
                flat: true
                enabled: !robot.busy
                onClicked: robot.reloadSettings()
            }
            Item { Layout.fillWidth: true }
            Button {
                text: qsTr("Apply")
                enabled: drawer.dirty && !robot.busy
                onClicked: robot.applySettings(drawer.draft, false)
            }
            Button {
                text: qsTr("Save to file")
                highlighted: true
                enabled: !robot.busy
                onClicked: robot.applySettings(drawer.draft, true)
            }
        }
    }
}
