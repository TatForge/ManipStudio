import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import QtCharts

// Plots of the last trajectory (moments, angles, speeds, accelerations) with
// a time cursor and a timeline that drives the 3D view.
Rectangle {
    id: panel
    color: Theme.panel
    border.color: Theme.panelBorder
    radius: Theme.radius

    readonly property var kinds: [
        { key: "moment", label: qsTr("Moments"), unit: qsTr("N·m") },
        { key: "position", label: qsTr("Angles"), unit: "°" },
        { key: "velocity", label: qsTr("Speeds"), unit: qsTr("°/s") },
        { key: "acceleration", label: qsTr("Accelerations"), unit: qsTr("°/s²") }
    ]
    property int kindIndex: 0
    readonly property string kind: kinds[kindIndex].key
    property var shown: [true, true, true, true, true, true]
    property bool showLimits: true

    property var curves: []         // one LineSeries per joint
    property var limitLines: []     // +max and -max per joint (moments only)
    property var cursor: null

    function refresh() {
        if (curves.length === 0)
            return
        // the scale follows the data; a limit line shows once a curve gets near it
        const range = robot.seriesRange(kind)
        const lo = range.yMin, hi = range.yMax
        const limits = robot.maxMoments
        const withLimits = kind === "moment" && showLimits
        for (let j = 0; j < 6; j++) {
            robot.fillSeries(curves[j], kind, j)
            curves[j].visible = shown[j]
            for (let s = 0; s < 2; s++) {
                const line = limitLines[2 * j + s]
                line.clear()
                line.visible = withLimits && shown[j] && limits.length === 6
                if (line.visible) {
                    const y = (s === 0 ? 1 : -1) * limits[j]
                    line.append(0, y)
                    line.append(Math.max(range.xMax, 0.01), y)
                }
            }
        }
        const pad = Math.max((hi - lo) * 0.08, 1)
        axisY.min = lo - pad
        axisY.max = hi + pad
        axisY.applyNiceNumbers()
        axisX.min = 0
        axisX.max = Math.max(range.xMax, 0.01)
        axisX.applyNiceNumbers()
        moveCursor()
    }

    function moveCursor() {
        if (!cursor)
            return
        cursor.clear()
        if (robot.sampleCount === 0)
            return
        cursor.append(robot.time, axisY.min)
        cursor.append(robot.time, axisY.max)
    }

    Connections {
        target: robot
        function onTrajectoryChanged() { panel.refresh() }
        function onProgressChanged() { panel.moveCursor() }
        function onReadyChanged() { panel.refresh() }
    }
    onKindChanged: refresh()
    onShownChanged: refresh()
    onShowLimitsChanged: refresh()

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Theme.gap
        spacing: 4

        RowLayout {
            Layout.fillWidth: true
            spacing: 6
            Repeater {
                model: panel.kinds
                delegate: Chip {
                    required property var modelData
                    required property int index
                    text: modelData.label
                    checked: panel.kindIndex === index
                    onClicked: panel.kindIndex = index
                }
            }
            Item { Layout.fillWidth: true; Layout.minimumWidth: 8 }
            Repeater {
                model: 6
                delegate: Chip {
                    required property int index
                    text: "q" + (index + 1)
                    tint: Theme.jointColors[index]
                    checked: panel.shown[index]
                    onClicked: {
                        const s = panel.shown.slice()
                        s[index] = !s[index]
                        panel.shown = s
                    }
                }
            }
            Chip {
                visible: panel.kind === "moment"
                text: qsTr("limits")
                tint: Theme.textDim
                checked: panel.showLimits
                onClicked: panel.showLimits = !panel.showLimits
            }
            Chip {
                text: "CSV"
                enabled: robot.sampleCount > 0
                onClicked: saveDialog.open()
            }
        }

        ChartView {
            id: chart
            Layout.fillWidth: true
            Layout.fillHeight: true
            theme: ChartView.ChartThemeDark
            backgroundColor: "transparent"
            plotAreaColor: "transparent"
            legend.visible: false
            antialiasing: true
            margins { left: 0; right: 0; top: 0; bottom: 0 }

            ValuesAxis {
                id: axisX
                min: 0
                max: 1
                titleText: qsTr("t, s")
                labelFormat: "%.2f"
                gridLineColor: Theme.panelBorder
                labelsColor: Theme.textDim
                titleBrush: Theme.textDim
            }
            ValuesAxis {
                id: axisY
                min: -1
                max: 1
                titleText: panel.kinds[panel.kindIndex].unit
                labelFormat: "%.1f"
                gridLineColor: Theme.panelBorder
                labelsColor: Theme.textDim
                titleBrush: Theme.textDim
            }

            Component.onCompleted: {
                const c = [], l = []
                for (let j = 0; j < 6; j++) {
                    const s = createSeries(ChartView.SeriesTypeLine, "q" + (j + 1), axisX, axisY)
                    s.color = Theme.jointColors[j]
                    s.width = 2
                    c.push(s)
                }
                for (let j = 0; j < 6; j++)
                    for (let k = 0; k < 2; k++) {
                        const s = createSeries(ChartView.SeriesTypeLine, "limit" + j + k, axisX, axisY)
                        s.color = Theme.jointColors[j]
                        s.width = 1
                        s.style = Qt.DashLine
                        l.push(s)
                    }
                const cur = createSeries(ChartView.SeriesTypeLine, "cursor", axisX, axisY)
                cur.color = Theme.text
                cur.width = 1
                panel.curves = c
                panel.limitLines = l
                panel.cursor = cur
                panel.refresh()
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.gap
            Button {
                text: robot.playing ? qsTr("Pause") : qsTr("Play")
                flat: true
                enabled: robot.sampleCount > 0 && !robot.busy
                onClicked: robot.playing ? robot.stop() : robot.play()
            }
            Slider {
                id: timeline
                Layout.fillWidth: true
                from: 0
                to: Math.max(robot.duration, 0.01)
                enabled: robot.sampleCount > 0 && !robot.busy
                value: robot.time
                onMoved: robot.seek(value)
            }
            Text {
                text: robot.time.toFixed(2) + " / " + robot.duration.toFixed(2) + qsTr(" s")
                color: Theme.textDim
                font.family: Theme.mono
            }
        }
    }

    Text {
        anchors.centerIn: parent
        visible: robot.momentCount === 0
        text: qsTr("Run a MoveJ to see the trajectory plots here")
        color: Theme.textDim
    }

    FileDialog {
        id: saveDialog
        title: qsTr("Save trajectory")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "csv"
        nameFilters: [qsTr("CSV (*.csv)")]
        onAccepted: {
            if (!robot.exportCsv(selectedFile))
                console.warn("CSV export failed:", selectedFile)
        }
    }
}
