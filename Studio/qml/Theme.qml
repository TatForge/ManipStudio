pragma Singleton
import QtQuick

QtObject {
    readonly property color window: "#15181d"
    readonly property color panel: "#1d2128"
    readonly property color panelBorder: "#2a2f38"
    readonly property color text: "#e6e9ef"
    readonly property color textDim: "#8b93a1"
    readonly property color accent: "#4fa3ff"
    readonly property color good: "#4cc38a"
    readonly property color warn: "#f0b429"
    readonly property color bad: "#ef5b5b"
    // axis colours, also used by the 3D view
    readonly property color axisX: "#ef5b5b"
    readonly property color axisY: "#4cc38a"
    readonly property color axisZ: "#4fa3ff"
    // one colour per joint in plots and tables
    readonly property var jointColors: ["#4fa3ff", "#f08a24", "#4cc38a", "#ef5b5b", "#b58cf5", "#f0d43a"]

    readonly property int radius: 8
    readonly property int gap: 12
    readonly property string mono: "monospace"
}
