import QtQuick

// Stand-in for the 3D view (step 2): a top-down sketch of the joint origins.
Rectangle {
    color: Theme.window
    border.color: Theme.panelBorder
    radius: Theme.radius

    Canvas {
        id: canvas
        anchors.fill: parent
        anchors.margins: Theme.gap
        property real pxPerMetre: Math.min(width, height) / 2.4   // +-1.2 m

        onPaint: {
            const ctx = getContext("2d")
            ctx.reset()
            const s = pxPerMetre
            const cx = width / 2, cy = height / 2
            ctx.strokeStyle = Theme.panelBorder
            ctx.lineWidth = 1
            for (let r = 0.25; r <= 1.0; r += 0.25) {
                ctx.beginPath(); ctx.arc(cx, cy, r * s, 0, 2 * Math.PI); ctx.stroke()
            }
            ctx.lineWidth = 2
            ctx.strokeStyle = Theme.axisX; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + 0.2 * s, cy); ctx.stroke()
            ctx.strokeStyle = Theme.axisY; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - 0.2 * s); ctx.stroke()

            const f = robot.frames
            if (!f || f.length === 0)
                return
            ctx.strokeStyle = Theme.accent
            ctx.lineWidth = 4
            ctx.beginPath()
            ctx.moveTo(cx, cy)
            for (let i = 0; i < f.length; i++)
                ctx.lineTo(cx + f[i].x * s, cy - f[i].y * s)
            ctx.stroke()
            ctx.fillStyle = Theme.text
            for (let i = 0; i < f.length; i++) {
                ctx.beginPath(); ctx.arc(cx + f[i].x * s, cy - f[i].y * s, 4, 0, 2 * Math.PI); ctx.fill()
            }
        }
        Connections {
            target: robot
            function onPoseChanged() { canvas.requestPaint() }
        }
    }

    Text {
        text: qsTr("Вид сверху (X вправо, Y вверх) · 3D-вид — следующий шаг")
        color: Theme.textDim
        anchors { left: parent.left; bottom: parent.bottom; margins: Theme.gap }
    }
}
