function load_config() {
    $.ajax({
        url: "/trackbar-config",
        method: "get",
        dataType: "json",
        success: function (data) {
            for (let i = 0; i < 6; ++i) {
                let text = "hsv"[i % 3] + " " + (i < 3 ? "L" : "H")

                let val = data[text];
                $(`#hsv${i}`).val(val)
                let e = $(`[for^=hsv${i}]`)
                e.text(e.text().split(":")[0] + ": " + val)
            }
        }
    })
}

function save_config() {
    let data = { }

    for (let i = 0; i < 6; ++i) {
        let text = "hsv"[i % 3] + " " + (i < 3 ? "L" : "H")

        data[text] = parseInt($(`#hsv${i}`).val())
    }

    $.ajax({
        url: "/trackbar-config",
        method: "post",
        data: JSON.stringify(data),
        contentType: "application/json"
    })
}

function calibrate_camera(){
    $.ajax({
        url: "/calibrate-camera",
        method: "post",
        success: function (msg, status, jqXHR) {
            $("#camera_calibration_response").text("calibration success")
        },
        error:function (msg, status, jqXHR) {
            $("#camera_calibration_response").text("calibration failed")
        }
    })
}

$(document).ready(_ => {
    for (let i = 0; i < 6; ++i) {
        let text = "hsv"[i % 3] + " " + (i < 3 ? "L" : "H")
        $("#calibrate").append(
`<label for="hsv${i}">${text}: 0</label>
<input type="range" min="0" max="255" id="hsv${i}" style="width: 50vh">`
        )
        $(`#hsv${i}`)[0].addEventListener("input", _ => {
            let e = $(`[for^=hsv${i}]`)
            e.text(e.text().split(":")[0] + ": " + $(`#hsv${i}`).val())
            save_config();
        });
    }

    load_config();
});
