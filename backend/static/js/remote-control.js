class WSHandler {
    constructor() {
        this.client_id = Date.now()
        this.ws = null
    }

    connect() {
        this.ws = new WebSocket(`ws://${window.location.hostname}/ws/${this.client_id}`);
        this.ws.onmessage = function (event) {
            console.log(event.data)
        };
    }
    status() {
        if (this.ws === null) {
            return WebSocket.CLOSED //if no ws initialized, defualt is closed state
        }
        return this.ws.readyState
    }
    send(data) {
        if (this.ws !== null) {
            this.ws.send(data)
        }
    }
}

$(document).ready(function () {
    var ws = new WSHandler()
    document.querySelector("#ws-id").textContent = ws.client_id;

    //ws watchdog
    setInterval(function () {
        let ws_status = $("#ws-status")

        if (ws.status() === WebSocket.OPEN) {
            if (ws_status.text() !== "OPEN") {
                $("#video").html(`
                    <h2>depth feed</h2>
                    <img src="/depth-feed" width="100%">

                    <h2>color feed</h2>
                    <img src="/color-feed" width="100%">
                `);
            }
        } else {
            $("#video").html("");
            window.stop(); // haha ebig hakke ::--DD
        }

        switch (ws.status()) {
            case WebSocket.CLOSED:
                ws_status.text("CLOSED")
                ws_status[0].setAttribute("class", "closed");
                ws.connect()
                break
            case WebSocket.CLOSING:
                ws_status.text("CLOSING")
                ws_status[0].setAttribute("class", "closed");
                break
            case WebSocket.CONNECTING:
                ws_status.text("CONNECTING")
                ws_status[0].setAttribute("class", "connecting");
                break
            case WebSocket.OPEN:
                ws_status.text("OPEN")
                ws_status[0].setAttribute("class", "connected");
                break
            default:
                ws_status.text("unknown")
                ws_status[0].setAttribute("class", "closed");
        }

    }, 100)

    function sendMessage(data) {
        ws.send(data)
    }

    var pid_ar = ["p", "i", "d", "setpoint"];
    pid_ar.forEach(v => {
        let e = $(`#pid_${v}`);
        e.change(function() {
            let v = e.val();
            let data = {"pid": pid_ar.map(v_ => parseFloat($(`#pid_${v_}`).val()))}
            console.log(data)
            sendMessage(JSON.stringify(data))
        });
    })

    var motors_enabled = false;
    var drive_enabled = false;
    var speed = 0;
    var thrower = 0;

    var movement = { "speed": 0, "direction": 0, "turn": 0, "thrower": 0, "enable": motors_enabled }
    var keys_pressed = { "up": false, "down": false, "left": false, "right": false, "turnL": false, "turnR": false }

    const speed_slider = $('#speed_slider')[0];
    speed_slider.value = speed
    speed_slider.addEventListener('input', updateSpeedValue); //Add listener if slider value changed 
    $("#speed_slider_text").text(speed);

    function updateSpeedValue(e) {
        speed = parseInt(e.target.value)
        $("#speed_slider_text").text(speed);
        updateMotorValues()
    }

    const thrower_slider = $('#thrower_slider')[0];
    thrower_slider.value = thrower
    thrower_slider.addEventListener('input', updateThrowerValue); //Add listener if slider value changed 
    $("#thrower_slider_text").text(thrower);

    function updateThrowerValue(e) {
        thrower = parseInt(e.target.value)
        $("#thrower_slider_text").text(thrower);
        updateMotorValues()
    }

    //called by every function that changes motor values
    function updateMotorValues() {
        if (!motors_enabled) {
            movement["speed"] = 0
            movement["turn"] = 0
            movement["thrower"] = 0
            movement["direction"] = 0
        } else {
            movement_calculator()
            movement["thrower"] = thrower
        }
        movement["enable"] = motors_enabled
        movement["drive_enable"] = drive_enabled
        sendMessage(JSON.stringify(movement))
    }

    //calculate speed and direction according to pressed keys
    function movement_calculator() {
        var x = 0;
        var y = 0;
        if (keys_pressed["up"]) {
            x += 1;
        }
        if (keys_pressed["down"]) {
            x -= 1;
        }
        if (keys_pressed["right"]) {
            y += 1;
        }
        if (keys_pressed["left"]) {
            y -= 1;
        }
        var rad = Math.atan2(y, x);
        movement["direction"] = Math.round(rad * (180 / Math.PI))

        if (x === 0 && y === 0) {
            movement["speed"] = 0
        } else {
            movement["speed"] = speed
        }

        movement["turn"] = keys_pressed["turnL"] ? speed : 0 + keys_pressed["turnR"] ? -speed : 0
    }

    function setKeyActive(key, val) { //sets keys that are pressed active and not pressed keys non active
        if (val) {
            $("#key_" + key).addClass("active")
        } else {
            $("#key_" + key).removeClass("active")
        }
    }

    function setKeyPressed(event, set_val) {
        let arrow = event.key.toLowerCase().split("arrow")
        if (arrow.length === 2) { //has length 2 only if arraow keys pressed
            keys_pressed[arrow[1]] = set_val;
            setKeyActive(arrow[1], set_val)
            updateMotorValues()
        }
        switch (event.key) { //handle keys for turn movement
            case "q":
                keys_pressed["turnL"] = set_val
                setKeyActive("turnL", set_val)
                updateMotorValues()
                break
            case "e":
                keys_pressed["turnR"] = set_val
                setKeyActive("turnR", set_val)
                updateMotorValues()
        }
    }

    $(document).keydown(function (event) {
        setKeyPressed(event, true)
    })

    $(document).keyup(function (event) {
        setKeyPressed(event, false)
    })

    $(document).keypress(function (event) {
        if (event.key == "f") {
            main_toggle()
        }
    })

    $("#main_switch").click(function () {
        main_toggle()
    })

    $("#drive_switch").click(function () {
        let e = $(this)
        const en = "main_enabled"
        const dis = "main_disabled"
        if (e.hasClass(dis)) {
            e.removeClass(dis)
            e.addClass(en)
            e.text("Drive ON")
            drive_enabled = true;
        } else {
            e.removeClass(en)
            e.addClass(dis)
            e.text("Drive OFF")
            drive_enabled = false;
        }
        updateMotorValues();
    })

    function send_show_mask(){
        $.ajax({
            url: "/set_debug_color_mask",
            method: "post",
            data: JSON.stringify({state:$("#show_mark_switch").text()})
        })
    }

    send_show_mask()
    $("#show_mark_switch").click(function () {
        if ($(this).text() == "ON") {
            $(this).text("OFF")
        } else {
            $(this).text("ON")
        }
        send_show_mask()
    })

    const control_keys = ["turnL", "turnR", "left", "right", "up", "down"]
    control_keys.forEach(e => { //set mouse click events on all of the buttons on html
        $("#key_" + e).mousedown(function () {
            keys_pressed[e] = true
            updateMotorValues()
        })
        $("#key_" + e).mouseup(function () {
            keys_pressed[e] = false
            updateMotorValues()
        })
    });

    function main_toggle() {  //main switch and attributes
        const en = "main_enabled"
        const dis = "main_disabled"
        let main_switch = $("#main_switch")
        if (main_switch.hasClass(dis)) {
            main_switch.removeClass(dis)
            main_switch.addClass(en)
            main_switch.text("ON")
            motors_enabled = true;
        } else {
            main_switch.removeClass(en)
            main_switch.addClass(dis)
            main_switch.text("OFF")
            motors_enabled = false;
        }
        updateMotorValues()
    }

    $("#target_goal").change(_ => {
        let val = $("#target_goal").val()
        let data = {"target_goal": val}
        $.ajax({
            url: "/set-target-goal",
            method: "post",
            data: JSON.stringify(data),
            contentType: "application/json"
        })
    })
});