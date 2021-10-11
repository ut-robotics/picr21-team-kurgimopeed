
$(document).ready(_ => {
    for (let i = 0; i < 6; ++i) {
        let text = "hsv"[i % 3] + " " + (i < 3 ? "L" : "H")
        $("#calibrate").append(
`<label for="hsv${i}">${text}: 0</label>
<input type="range" min="0" max="255" id="hsv${i}">`
        )
        $(`#hsv${i}`).change(_ => {
            let e = $(`[for^=hsv${i}]`)
            console.log($(`#hsv${i}`).val())
            e.text(e.text().split(":")[0] + ": " )
        });
    }
});
