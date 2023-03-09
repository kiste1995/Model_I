/**
 * jQuery Line Progressbar2
 * Author: KingRayhan<rayhan095@gmail.com>
 * Author URL: https://electronthemes.com
 * Version: 1.1.2
 */

;(function($) {
    'use strict'

    $.fn.LineProgressbar2 = function(options) {
        options = $.extend(
            {
                percentage: 100,
                ShowProgressCount: true,
                duration: 1000,
                unit: '%',
                animation: false,

                // Styling Options
                fillBackgroundColor: '#3498db',
                backgroundColor: '#EEEEEE',
                radius: '0px',
                height: '10px',
                width: '100%',
            },
            options
        )

        $.options = options
        return this.each(function(index, el) {
            // Markup
            $(el).html(
                '<div class="progressbar2"><div class="proggress2"></div><div class="percentCount2"></div></div>'
            )

            var progressFill2 = $(el).find('.proggress2')
            var progressBar2 = $(el).find('.progressbar2')

            progressFill2.css({
                backgroundColor: options.fillBackgroundColor,
                height: options.height,
                borderRadius: options.radius,
            })
            progressBar2.css({
                width: options.width,
                backgroundColor: options.backgroundColor,
                borderRadius: options.radius,
            })

            /**
             * Progress with animation
             */
            if (options.animation) {
                // Progressing2
                progressFill2.animate(
                    {
                        width: options.percentage + '%',
                    },
                    {
                        step: function(x) {
                            if (options.ShowProgressCount) {
                                $(el)
                                    .find('.percentCount')
                                    .text(Math.round(x) + options.unit)
                            }
                        },
                        duration: options.duration,
                    }
                )
            } else {
                // Without animation
                progressFill2.css('width', options.percentage + '%')
                $(el)
                    .find('.percentCount')
                    .text(Math.round(options.percentage) + '%')
            }
        })
    }
})(jQuery)

$('[line-progressbar2]').each(function() {
    var $this = $(this)
    function LineProgressing2() {
        $this.LineProgressbar2({
            percentage: $this.data('percentage'),
            unit: $this.data('unit'),
            animation: $this.data('animation'),
            ShowProgressCount: $this.data('showcount'),
            duration: $this.data('duration'),
            fillBackgroundColor: $this.data('progress-color'),
            backgroundColor: $this.data('bg-color'),
            radius: $this.data('radius'),
            height: $this.data('height'),
            width: $this.data('width'),
        })
    }
    var loadOnce = 0
    $this.waypoint(
        function() {
            loadOnce += 1
            if (loadOnce < 2) {
                LineProgressing()
            }
        },
        { offset: '100%', triggerOnce: true }
    )
})
