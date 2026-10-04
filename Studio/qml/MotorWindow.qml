import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtCharts

// Motor set-up and test window: CAN connection, one ODrive axis per joint,
// configuration, enable / jog / move, live plots, stop.
ApplicationWindow {
    id: win
    width: 1150
    height: 780
    minimumWidth: 900
    minimumHeight: 600
    title: qsTr("Motors") + " — ManipStudio"
    color: Theme.window

    Material.theme: Material.Dark
    Material.accent: Theme.accent
    Material.primary: Theme.panel

    property int selected: 0
    property string lastError: ""
    readonly property var axis: selected < motors.axes.length ? motors.axes[selected] : null

    function joints() { motors.setJointLimits(robot.limits) }
    Component.onCompleted: joints()
    Connections {
        target: robot
        function onReadyChanged() { win.joints() }
    }
    Connections {
        target: motors
        function onErrorOccurred(message) { win.lastError = message }
        function onConnectedChanged() { win.lastError = "" }
    }
    onClosing: motors.stopAll()

    function fmt(v, d) { return Number(v).toFixed(d) }

    header: ToolBar {
        Material.background: Theme.panel
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: Theme.gap
            anchors.rightMargin: Theme.gap
            spacing: 8
            Text { text: "CAN"; color: Theme.textDim }
            ComboBox {
                id: backendBox
                model: motors.backends
                enabled: !motors.connected
                Layout.preferredWidth: 150
                onCurrentTextChanged: interfaceBox.refresh()
            }
            ComboBox {
                id: interfaceBox
                enabled: !motors.connected
                Layout.preferredWidth: 150
                function refresh() { model = motors.interfaces(backendBox.currentText) }
                Component.onCompleted: refresh()
            }
            Chip { text: "⟳"; enabled: !motors.connected; onClicked: interfaceBox.refresh() }
            ComboBox {
                id: bitrateBox
                model: ["125000", "250000", "500000", "1000000"]
                currentIndex: 1
                enabled: !motors.connected
                Layout.preferredWidth: 120
            }
            Button {
                text: motors.connected ? qsTr("Disconnect") : qsTr("Connect")
                highlighted: !motors.connected
                onClicked: motors.connected ? motors.disconnectBus()
                                            : motors.connectBus(backendBox.currentText, interfaceBox.currentText, Number(bitrateBox.currentText))
            }
            Rectangle { width: 10; height: 10; radius: 5; color: motors.connected ? Theme.good : Theme.textDim }
            Text {
                text: motors.connected ? motors.busName : qsTr("not connected")
                color: Theme.textDim
                elide: Text.ElideRight
                Layout.fillWidth: true
            }
            Text { text: qsTr("Motors"); color: Theme.textDim }
            SpinBox {
                from: 1; to: 6
                value: motors.axisCount
                onValueModified: motors.axisCount = value
                Layout.preferredWidth: 110
            }
            Chip { text: qsTr("Save settings"); onClicked: motors.saveConfig() }
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: Theme.gap
        spacing: Theme.gap

        // ------------------------------------------------ all axes
        Card {
            title: qsTr("Axes")
            Layout.preferredWidth: 330
            Layout.fillHeight: true
            Repeater {
                model: motors.axes
                delegate: Rectangle {
                    required property var modelData
                    Layout.fillWidth: true
                    implicitHeight: row.implicitHeight + 12
                    radius: 4
                    color: win.selected === modelData.index ? Qt.rgba(1, 1, 1, 0.07) : "transparent"
                    border.color: win.selected === modelData.index ? Theme.accent : Theme.panelBorder
                    MouseArea { anchors.fill: parent; onClicked: win.selected = modelData.index }
                    ColumnLayout {
                        id: row
                        anchors { left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter; margins: 8 }
                        spacing: 2
                        RowLayout {
                            Text { text: modelData.name; color: Theme.jointColors[modelData.index]; font.bold: true }
                            Rectangle {
                                width: 8; height: 8; radius: 4
                                color: !modelData.online ? Theme.textDim : modelData.error !== "" ? Theme.bad
                                     : modelData.closedLoop ? Theme.good : Theme.warn
                            }
                            Text {
                                text: modelData.online ? modelData.stateName : qsTr("offline") + " (node " + modelData.node + ")"
                                color: Theme.textDim; font.pixelSize: 11
                                Layout.fillWidth: true
                            }
                            Text {
                                text: modelData.hasPosition ? win.fmt(modelData.position, 2) + "°" : "—"
                                color: Theme.text; font.family: Theme.mono
                            }
                        }
                        Text {
                            visible: modelData.online
                            text: win.fmt(modelData.velocity, 1) + " °/s   " + win.fmt(modelData.current, 2) + " A   "
                                  + win.fmt(modelData.busVoltage, 1) + " V"
                            color: Theme.textDim; font.family: Theme.mono; font.pixelSize: 11
                        }
                        Text {
                            visible: modelData.error !== ""
                            text: modelData.error + " " + modelData.errorText
                            color: Theme.bad; font.pixelSize: 11
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                    }
                }
            }
            Text {
                Layout.fillWidth: true
                Layout.topMargin: Theme.gap
                wrapMode: Text.Wrap
                color: Theme.textDim
                font.pixelSize: 11
                text: qsTr("Joint angle = direction × (motor turns − zero) / gear × 360°. Set the ODrive watchdog: the motors keep the last command if this program stops talking.")
            }
        }

        // ------------------------------------------------ selected axis
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: Theme.gap

            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gap

                Card {
                    title: qsTr("Settings") + (win.axis ? " " + win.axis.name : "")
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignTop
                    GridLayout {
                        columns: 4
                        columnSpacing: 8
                        rowSpacing: 2
                        enabled: win.axis !== null
                        Text { text: qsTr("CAN node"); color: Theme.textDim }
                        NumberField {
                            value: win.axis ? win.axis.node : 0
                            onEdited: v => motors.setAxisConfig(win.selected, { node: v })
                        }
                        Text { text: qsTr("Gear ratio"); color: Theme.textDim }
                        NumberField {
                            value: win.axis ? win.axis.gear : 1
                            onEdited: v => motors.setAxisConfig(win.selected, { gear: v })
                        }
                        Text { text: qsTr("Direction"); color: Theme.textDim }
                        Row {
                            spacing: 4
                            Chip { text: "+1"; checked: win.axis && win.axis.direction === 1; onClicked: motors.setAxisConfig(win.selected, { direction: 1 }) }
                            Chip { text: "−1"; checked: win.axis && win.axis.direction === -1; onClicked: motors.setAxisConfig(win.selected, { direction: -1 }) }
                        }
                        Text { text: qsTr("Zero, turns"); color: Theme.textDim }
                        RowLayout {
                            NumberField {
                                value: win.axis ? win.axis.zero : 0
                                onEdited: v => motors.setAxisConfig(win.selected, { zero: v })
                            }
                            Chip { text: qsTr("Here"); enabled: win.axis && win.axis.hasPosition; onClicked: motors.setZeroHere(win.selected) }
                        }
                        Text { text: qsTr("Speed, °/s"); color: Theme.textDim }
                        NumberField {
                            value: win.axis ? win.axis.maxSpeed : 0
                            onEdited: v => motors.setAxisConfig(win.selected, { maxSpeed: v })
                        }
                        Text { text: qsTr("Accel., °/s²"); color: Theme.textDim }
                        NumberField {
                            value: win.axis ? win.axis.maxAccel : 0
                            onEdited: v => motors.setAxisConfig(win.selected, { maxAccel: v })
                        }
                        Text { text: qsTr("Current, A"); color: Theme.textDim }
                        NumberField {
                            value: win.axis ? win.axis.currentLimit : 0
                            onEdited: v => motors.setAxisConfig(win.selected, { currentLimit: v })
                        }
                        Text { text: qsTr("Range, °"); color: Theme.textDim }
                        Text {
                            text: win.axis ? win.fmt(win.axis.min, 0) + " … " + win.fmt(win.axis.max, 0) : ""
                            color: Theme.text; font.family: Theme.mono
                        }
                    }
                }

                Card {
                    id: testCard
                    title: qsTr("Test")
                    Layout.preferredWidth: 330
                    Layout.alignment: Qt.AlignTop
                    readonly property bool online: win.axis !== null && win.axis.online
                    readonly property bool closed: win.axis !== null && win.axis.closedLoop
                    Flow {
                        Layout.fillWidth: true
                        spacing: 6
                        Button {
                            text: qsTr("Calibrate")
                            enabled: testCard.online && win.axis.state === 1
                            onClicked: calibrateDialog.open()
                        }
                        Button {
                            text: qsTr("Enable")
                            highlighted: true
                            enabled: testCard.online && !testCard.closed
                            onClicked: motors.enable(win.selected)
                        }
                        Button {
                            text: qsTr("Disable")
                            enabled: testCard.online
                            onClicked: motors.disable(win.selected)
                        }
                        Button {
                            text: qsTr("Clear errors")
                            enabled: testCard.online && win.axis.error !== ""
                            onClicked: motors.clearErrors(win.selected)
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        enabled: win.axis !== null && win.axis.closedLoop
                        Text { text: qsTr("Jog"); color: Theme.textDim; Layout.preferredWidth: 60 }
                        Repeater {
                            model: [-1, 1]
                            delegate: Button {
                                required property var modelData
                                text: modelData < 0 ? "◀ −" : "+ ▶"
                                Layout.fillWidth: true
                                // hold to run: repeated while pressed, the motor stops if the repeats stop
                                onPressed: motors.jog(win.selected, modelData)
                                onReleased: motors.jog(win.selected, 0)
                                onCanceled: motors.jog(win.selected, 0)
                                Timer {
                                    interval: 100
                                    repeat: true
                                    running: parent.pressed
                                    onTriggered: motors.jog(win.selected, parent.modelData)
                                }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        enabled: win.axis !== null && win.axis.closedLoop
                        Text { text: qsTr("Go to, °"); color: Theme.textDim; Layout.preferredWidth: 60 }
                        NumberField { id: targetField; value: 0; Layout.fillWidth: true }
                        Button {
                            text: qsTr("Go")
                            onClicked: motors.moveTo(win.selected, Number(targetField.text))
                        }
                        Button {
                            text: "0°"
                            onClicked: motors.moveTo(win.selected, 0)
                        }
                    }
                }
            }

            // ------------------------------------------------ live plots
            Card {
                id: plots
                title: qsTr("Live")
                Layout.fillWidth: true
                Layout.fillHeight: true
                property string kind: "position"

                Row {
                    spacing: 6
                    Chip { text: qsTr("Position"); checked: plots.kind === "position"; onClicked: plots.kind = "position" }
                    Chip { text: qsTr("Speed"); checked: plots.kind === "velocity"; onClicked: plots.kind = "velocity" }
                    Chip { text: qsTr("Current"); checked: plots.kind === "current"; onClicked: plots.kind = "current" }
                }
                ChartView {
                    id: chart
                    Layout.fillWidth: true
                    Layout.preferredHeight: Math.max(160, plots.height - 80)
                    antialiasing: true
                    theme: ChartView.ChartThemeDark
                    backgroundColor: "transparent"
                    legend.visible: false
                    margins { left: 0; right: 0; top: 0; bottom: 0 }
                    ValueAxis { id: xAxis; gridLineColor: Theme.panelBorder; labelsColor: Theme.textDim; labelFormat: "%.0f" }
                    ValueAxis { id: yAxis; gridLineColor: Theme.panelBorder; labelsColor: Theme.textDim }
                    LineSeries { id: actual; axisX: xAxis; axisY: yAxis; color: Theme.accent; width: 2 }
                    LineSeries { id: target; axisX: xAxis; axisY: yAxis; color: Theme.warn; width: 1; style: Qt.DashLine }
                }
                Timer {
                    interval: 100
                    repeat: true
                    running: win.visible && motors.connected
                    onTriggered: {
                        const kinds = plots.kind === "position" ? ["position", "target"] : [plots.kind]
                        motors.fillSeries(actual, win.selected, plots.kind)
                        if (plots.kind === "position")
                            motors.fillSeries(target, win.selected, "target")
                        else
                            target.clear()
                        const r = motors.seriesRange(win.selected, kinds)
                        xAxis.min = r.xMin; xAxis.max = r.xMax
                        yAxis.min = r.yMin; yAxis.max = r.yMax
                    }
                }
            }
        }
    }

    footer: Rectangle {
        color: Theme.panel
        implicitHeight: footRow.implicitHeight + 16
        RowLayout {
            id: footRow
            anchors.fill: parent
            anchors.leftMargin: Theme.gap
            anchors.rightMargin: Theme.gap
            spacing: Theme.gap
            Button {
                text: qsTr("STOP")
                font.bold: true
                font.pixelSize: 16
                Material.background: Theme.bad
                Material.foreground: "white"
                Layout.preferredWidth: 160
                enabled: motors.connected
                onClicked: motors.stopAll()
            }
            Button {
                text: qsTr("Disable all")
                enabled: motors.connected
                onClicked: motors.disableAll()
            }
            Text {
                text: win.lastError !== "" ? win.lastError : motors.status
                color: win.lastError !== "" ? Theme.bad : Theme.textDim
                elide: Text.ElideRight
                Layout.fillWidth: true
            }
            Shortcut { sequence: "Esc"; onActivated: motors.stopAll() }
            Text { text: qsTr("Esc = stop"); color: Theme.textDim; font.pixelSize: 11 }
        }
    }

    Dialog {
        id: calibrateDialog
        anchors.centerIn: parent
        modal: true
        title: qsTr("Calibrate %1?").arg(win.axis ? win.axis.name : "")
        standardButtons: Dialog.Ok | Dialog.Cancel
        Label {
            width: 360
            wrapMode: Text.Wrap
            text: qsTr("The motor will beep and turn by itself for several seconds. The joint must be free to move and nothing may be in its way.")
        }
        onAccepted: motors.calibrate(win.selected)
    }
}
