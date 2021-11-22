var threshold_values = {}

function load_config() {
    $.ajax({
        url: "/trackbar-config",
        method: "get",
        dataType: "json",
        success: function (data) {
            console.log(data)
            threshold_values = data;
            //debugger
            for ([key, values] of Object.entries(threshold_values)) {
                let group = `${key}_threshold`;
                $("#calibrate").append(`<div id="${group}"></div>`)
                let groupel = $(`#${group}`);
                groupel.hide();
                for (let i = 0; i < 6; ++i) {
                    groupel.append(`<input type="range" min="0" max="255" id="${group}_${i}" style="width: 50vh" value="${values[i]}">`)
                    $(`#${group}_${i}`)[0].addEventListener("input", _ => {
                        save_config()
                    })
                }
            }

            var first = `#${Object.keys(threshold_values)[0]}_threshold`;
            $(first).show();

            $("#threshold_config").change(_ => {
                $(first).hide()
                let val = `#${$("#threshold_config").val()}_threshold`;
                $(val).show();
                first = val;

                save_config();
            });
        }
    })
}

function save_config() {
    let data = {
        "threshold_config": $("#threshold_config").val(),
        "threshold_values": []
    }

    for (let i = 0; i < 6; ++i) {
        let element = `#${data.threshold_config}_threshold_${i}`;

        data["threshold_values"].push(parseInt($(element).val()))
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
    load_config();
});
