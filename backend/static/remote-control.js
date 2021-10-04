var movement = { "speed": 0, "direction": 0 }



$(document).ready(function () {

    var client_id = Date.now()
    document.querySelector("#ws-id").textContent = client_id;
    var ws = new WebSocket(`ws://${window.location.hostname}/ws/${client_id}`);
    ws.onmessage = function (event) {
        console.log(event.data)
    };
    function sendMessage(data) {
        ws.send(data)
    }

    var keys_pressed = { "up": false, "down": false, "left": false, "right": false }

    const input = $('#speed_slider')[0];
    var speed = parseInt(input.value);

    input.addEventListener('input', updateValue);

    function updateValue(e) {
        speed = parseInt(e.target.value)
    }

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
        sendMessage(JSON.stringify(movement))
    }

    $(document).keydown(function (event) {
        switch (event.key) {
            case "ArrowLeft":
                keys_pressed["left"] = true;
                break;
            case "ArrowRight":
                keys_pressed["right"] = true;
                break;
            case "ArrowUp":
                keys_pressed["up"] = true;
                break;
            case "ArrowDown":
                keys_pressed["down"] = true;
                break;
        }
        movement_calculator()
    })

    $(document).keyup(function (event) {
        switch (event.key) {
            case "ArrowLeft":
                keys_pressed["left"] = false;
                break;
            case "ArrowRight":
                keys_pressed["right"] = false;
                break;
            case "ArrowUp":
                keys_pressed["up"] = false;
                break;
            case "ArrowDown":
                keys_pressed["down"] = false;
                break;
        }
        movement_calculator()
    })
});