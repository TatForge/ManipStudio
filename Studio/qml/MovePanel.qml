import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// MoveJ: target joint angles, speed and acceleration.
Card {
    id: panel
    title: qsTr("MoveJ")

    property var target: [20, -40, 60, 10, 45, -30]

    GridLayout {
        columns: 3
        columnSpacing: 8
        rowSpacing: 2
        Layout.fillWidth: true

        Repeater {
            model: 6
            delegate: RowLayout {
                required property int index
                spacing: 4
                Text { text: "q" + (index + 1); color: Theme.textDim; font.family: Theme.mono }
                TextField {
                    Layout.preferredWidth: 64
                    text: panel.target[index]
                    validator: DoubleValidator { decimals: 2 }
                    horizontalAlignment: TextInput.AlignRight
                    onEditingFinished: {
                        const t = panel.target.slice()
                        t[index] = Number(text)
                        panel.target = t
                    }
                }
            }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        Text { text: qsTr("Скорость, °/с"); color: Theme.textDim; Layout.fillWidth: true }
        SpinBox { id: speed; from: 1; to: 360; value: 60; editable: true }
    }
    RowLayout {
        Layout.fillWidth: true
        Text { text: qsTr("Ускорение, °/с²"); color: Theme.textDim; Layout.fillWidth: true }
        SpinBox { id: accel; from: 1; to: 1000; value: 120; editable: true }
    }

    RowLayout {
        Layout.fillWidth: true
        Button {
            text: qsTr("Текущие")
            flat: true
            enabled: robot.ready
            onClicked: panel.target = robot.joints.map(v => Math.round(v * 100) / 100)
        }
        Item { Layout.fillWidth: true }
        Button {
            text: robot.playing ? qsTr("Стоп") : qsTr("Выполнить")
            highlighted: true
            enabled: robot.ready && !robot.busy
            onClicked: robot.playing ? robot.stop() : robot.moveJ(panel.target, speed.value, accel.value)
        }
    }
}
