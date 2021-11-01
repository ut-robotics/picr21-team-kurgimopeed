$(document).ready(function () {

$(".playmusic").click(function() {
    let song = $(this).attr('song')
    $.ajax({
        url: "/playmusic",
        method: "post",
        data: {
            song: song
        }
    })
})

$(".stopmusic").click(function() {
    $.ajax({
        url: "/stopmusic",
        method: "post"
    })
})

})