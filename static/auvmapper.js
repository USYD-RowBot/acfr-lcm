/**********************************************
 *
 * AUV Platform mapper
 *
 * Author:
 *      Ariell Friedman
 *      ariell.friedman@gmail.com
 *      25 AUG 2014
 *
 *********************************************/

/**
 * Constructor
 */

function auvmapper () {
    var _this = this;   // for convenient use in different contexts
    this.origin = [-33.889643, 151.193483]; // default origin ACFR
    this.maxposes = 100; // maximum number of poses in a platform track (to avoid memory/performance issues)

    // Initialise layers and info
    this.layers = {};
    this.layers.base = {};
    this.layers.overlays = {};
    this.info = {};
    this.dash = {};
    this.snap_to_track = {};
    this.autotrack_layer = {};

    /**
     * Intialise the map and layers
     * @param mapid
     */
    this.init = function(mapid) {
        this.mapid = mapid;
        this.fullsize();
        this.map = new L.Map(mapid, {
            center: new L.LatLng(this.origin[0],this.origin[1]),
            zoom: 18,
            maxZoom: 25,  // max zoom is more than native zoom - leads to pixelated tiles
            maxNativeZoom: 17,
            fullscreenControl: true
        });

        // add blank tile layer
        this.layers.base = {'None': L.tileLayer('')},
        // Add base layers
//        try {
//            this.layers.base = {
//                "Google": new L.Google('STREETMAP'),
//                "Satellite": new L.Google('HYBRID')
//            };
//            //this.map.addLayer(this.layers.base.Satellite); // load default map layer
//        }
//        catch(err) {
//            console.log(err);
//            this.layers.base = {};
//        }


        // Add layer control
        this.layerctl = new L.control.layers(this.layers.base, this.layers.overlays, {autoZIndex: true}).addTo(this.map);
        L.control.scale({imperial:false}).addTo(this.map);


        // Deactivate auto-extent on map drag
        this.map.on("dragstart",function(){
            _this.auto_extent ("all", false);
        });

        // Add permalink control
        //this.map.addControl(new L.Control.Permalink({useLocation: true}));


        // Add measurement control
        L.Control.measureControl().addTo(this.map);
        $(".leaflet-control-fullscreen-button").tooltip({placement:"right",trigger:"hover",container:"body"});
        // Add mouse position
        L.control.mousePosition().addTo(this.map);
        $(".leaflet-control-draw-measure").tooltip({placement:"right",trigger:"hover",container:"body"});



        /*
        // TEST: control to get latlng - WIP
        this.add_control("o",function(){
            _this.map.on('click', function(e) {
                alert("Lat, Lon : " + e.latlng.lat + ", " + e.latlng.lng)
            });
        },"Demo click latlng");
        */

        // Initialise layer group for auto tracking
        this.autotrack_layer = new L.featureGroup([]).addTo(this.map);
    }

    /**
     * Get load the mission layer from an ajax call.
     * If unsuccessful, it retries every 5 seconds.
     * @param layer
     */
    this.get_mission = function(layer, filepath, url, usroptions, origin){
        if (typeof usroptions == "undefined") usroptions = {};
        if (typeof origin == "undefined") origin = [0,0];
        var options = { color: 'red', weight: 1, opacity: 0.3, smoothFactor: 1};
        $.extend(options,usroptions);

        // set mission
        _this.layers.overlays[layer] = new L.Polyline([], options).addTo(_this.map);
        _this.layerctl.addOverlay(_this.layers.overlays[layer], layer);
        _this.layers.overlays[layer].bindPopup(layer);

        $.ajax({
            dataType: "jsonp",
            url: url,
            data: {filepath: filepath, olat: origin[0], olon: origin[1]},
            success: function (data) {
                $("#track-layers").prepend($("<li><i class='fa fa-slack'> Mission: "+layer+"</i></li>").click(function(){
                    _this.auto_extent ("all", false);
                    _this.map.fitBounds(_this.layers.overlays[layer].getBounds());
                }));
                // set mission
                _this.layers.overlays[layer].setLatLngs(data.latlngs);
                //_this.map.fitBounds(_this.layers.overlays[layer].getBounds());

                // set origin
                _this.origin = data.origin;
            },
            error : function (jqXHR, status, desc) {
                console.log("Cannot load mission: "+filepath,jqXHR);
                setTimeout(function(){_this.get_mission(layer, filepath, url, usroptions, origin)},5000); // try again in 5 seconds
            }
        });
    }


    /**
     * Load a geotiff from a url
     * @param layer: name of layer (no spaces)
     * @param url: url of geotiff
     */
    this.get_geotiff = function(layer, imgurl, url){
        $.ajax({
            dataType: "jsonp",
            url: url,
            data: {url: imgurl},
            success: function (data) {
                $("#track-layers").prepend($("<li><i class='fa fa-picture-o'> Img: "+layer+"</i></li>").click(function(){
                    _this.auto_extent ("all", false);
                    _this.map.fitBounds(L.latLngBounds(data.bounds));
                }));
//                _this.add_control ("<i class='fa fa-picture-o'></i>", function(){
//                    _this.auto_extent ("all", false);
//                    _this.map.fitBounds(L.latLngBounds(data.bounds));
//                }, "Reset zoom image ("+layer+")");
                // set geotiff
                _this.layers.overlays[layer] = new L.imageOverlay(data.imgurl, data.bounds);//.addTo(_this.map).bringToBack();
                _this.map.addLayer(_this.layers.overlays[layer], true);

                //_this.layers.overlays[layer].extend({onAdd: this.bringToBack});
                _this.layerctl.addOverlay(_this.layers.overlays[layer].bringToBack(), layer);
            },
            error : function (jqXHR, status, desc) {
                console.log("Cannot load geotiff: "+imgurl,jqXHR);
                setTimeout(function(){_this.get_geotiff(layer, imgurl, url)},5000); // try again in 5 seconds
            }
        });
    }

    this.get_kml = function(layer, filepath){

        _this.layers.overlays[layer] = new L.KML(filepath, {async: true});
        _this.layers.overlays[layer].addTo(this.map).on("loaded", function(e) {
            $("#track-layers").prepend($("<li><i class='fa fa-file-image-o'> KML: "+layer+"</i></li>").click(function(){
                _this.auto_extent ("all", false);
                _this.map.fitBounds(e.target.getBounds());
            }));
            _this.layerctl.addOverlay(_this.layers.overlays[layer].bringToBack(), layer);
        });
    }



    /**
     * Create pose tracking layer
     * @param platform
     * @param url
     * @param interval
     * @param trackoptions
     * @param markeroptions
     * @param showdash
     */
    this.add_posetracker = function(platform, url, usroptions, allowcontrol) {
        if (typeof usroptions == "undefined") usroptions = {};
        var dispoptions = {showdash: false, showrph: false, showbatt: false, showtrack: false, setwaypoint: false}; // default dispoptions
        $.extend(dispoptions,usroptions.dispoptions);  // extend to make sure all fields
        usroptions.dispoptions = dispoptions;  // copy to usroptions
        var options = {color: 'red', interval: 1000, size: 3, maxtracklen: 100};  // default other otpions
        $.extend(options,usroptions); // extend options

        var trackoptions={color: options.color, weight: 1, opacity: 0.9, smoothFactor: 1 },
            markeroptions = {color: options.color, weight: 2.5, fillColor: "black", fillOpacity: 0.5, opacity: 1, zIndexOffset: 1000},
            polyoptions = {color: options.color,weight:2.5,fillColor:"white",fillOpacity:0.5,opacity:1},
            uncmarkeroptions = {color: options.color, weight: 0.4, fillColor: options.color, fillOpacity: 0.2, opacity: 1, zIndexOffset: 1000};

        var tracklayer = platform+" track",
            unclayer = platform+" uncertainty";

        // add track layer
        if (options.dispoptions.showtrack) {
            this.layers.overlays[tracklayer] = new L.Polyline([], trackoptions).addTo(this.map);
            this.layerctl.addOverlay(this.layers.overlays[tracklayer], tracklayer);
            console.log(this.layers.overlays[tracklayer]);
            // Add fancy mid line markers
            //var markerid = 'circle-'+platform.replace(' ','_');
            //$(this.layers.overlays[tracklayer]._container).prepend('<marker id="'+markerid+'" markerWidth="6" markerHeight="6" refX="3" refY="3" markerUnits="userSpaceOnUse"><circle cx="3" cy="3" r="3" fill="'+options.color+'"></circle></marker>');
            //$(this.layers.overlays[tracklayer]._path).attr("marker-mid","url(#"+markerid+")");
        }

        // if size is an object, then draw a ship, otherwise draw a circle
        if (typeof options.size === "number") {
            this.layers.overlays[platform] = new L.circle(this.origin, options.size, markeroptions);
        }
        else if (typeof options.size === "object") {
            this.layers.overlays[platform] = new L.polygon([],polyoptions);
            this.layers.overlays[platform].poly = new getShipPoly(options.size, this.layers.overlays[platform]);
            this.layers.overlays[platform].poly.setLatLngHdg(0,new L.LatLng(this.origin[0],this.origin[1]));
        }
        // uncertainty layer
        this.layers.overlays[unclayer] = new L.circle(this.origin, 1, uncmarkeroptions);

        // Add popup
        this.layers.overlays[platform].bindPopup(platform);

        // add to layer group (platform & unc) to control
        var layergroup = new L.layerGroup([this.layers.overlays[unclayer],this.layers.overlays[platform]]);
        layergroup.addTo(this.map);
        this.layerctl.addOverlay(layergroup, platform);

        // initialise info panel
        this.snap_to_track[platform] = false;
        this.add_platform_controls(platform, tracklayer, markeroptions.color, options.dispoptions, allowcontrol);
        this.add_platform_dashboard(platform, markeroptions.color, options.dispoptions);

        // start position updater
        this.update_posetracker(tracklayer, unclayer, platform, url, options.interval, options.maxtracklen);
    }

    /**
     * Updates pose tracking layer at regular interval.
     * If fails, retries every 5 seconds.
     * @param tracklayer
     * @param platform
     * @param url
     * @param interval
     */
    this.update_posetracker = function( tracklayer, unclayer,  platform, url, interval, maxtracklen) {
        $.ajax({
            dataType: "jsonp",
            url: url,
            timeout: 20000, // sets timeout to 20 seconds
            success: function (data) {
                if (parseInt(_this.info[platform].data('msgts')) < parseInt(data.msgts)) {
                    _this.info[platform].data('msgts',data.msgts);
                    $(_this.info[platform]).parent().css('background-color','#FFF')

                    set_pose(platform, tracklayer, unclayer, maxtracklen, data.pose)

                    // If we are tracking, check bounds and move map to track items
                    if (_this.autotrack_layer.getLayers().length > 0) {
                        if (!_this.map.getBounds().contains(_this.autotrack_layer.getBounds())) {
                            // if tracking more than 1 layer, set bounds, otherwise simply pan map
                            //_this.map.fitBounds(_this.autotrack_layer.getBounds());
                            if (_this.autotrack_layer.getLayers().length > 1) _this.map.fitBounds(_this.autotrack_layer.getBounds());
                            else if (_this.autotrack_layer.getLayers()[0].hasOwnProperty("poly")) _this.map.panTo(_this.autotrack_layer.getLayers()[0].getLatLngs()[0]);
                            else _this.map.panTo(_this.autotrack_layer.getLayers()[0].getLatLng());
                        }
                    }
                    // update dashboard if it is visible (and exists)
                    if ($(_this.dash[platform]).is(":visible")) {
                        var pose = data.pose,
                            stat = data.stat,
                            alert = data.alert;
                        if ($(_this.dash[platform]).find(".hdg-rol").length > 0) {  //update roll-pitch-heading widget (if visible)
                            $(_this.dash[platform]).find(".hdg").val(pose.heading).trigger('change'); // update heading vis
                            $(_this.dash[platform]).find(".rol").val(pose.roll).trigger('change'); // update roll vis
                            $(_this.dash[platform]).find(".rol-canvas").css("top", 25 - pose.pitch * 50 / 100); // update pitch vis
                            $(_this.dash[platform]).find(".rph-info").html(formatdata({HDG: pose.heading, PITCH: pose.pitch, ROLL: pose.roll}));
                            delete pose.roll; delete pose.heading; delete pose.pitch;  // remove fields to avoid duplicate displays
                        }
                        if ($(_this.dash[platform]).find(".bat").length > 0) {  // update battery widget (if visible)
                            $(_this.dash[platform]).find(".bat").css("width", stat.bat + "%").html(stat.bat + "%");
                            delete stat.bat;  // remove fields to avoid duplicate displays
                        }

                        // Show remaining data on dashboard
                        $(_this.dash[platform]).find(".platform-alerts").html(formatdata(alert, "alerts"));
                        $(_this.dash[platform]).find(".platform-stat").html(formatdata(stat));
                        $(_this.dash[platform]).find(".platform-pose").html(formatdata(pose));
                        $(_this.info[platform]).html(""); // clear info panel
                    }
                    else {
                        // make info object by joining pose and stat (if stat exists)
                        var info = (data.hasOwnProperty("stat")) ? $.extend(data.pose, data.stat) : data.pose;
                        $(_this.info[platform]).html(formatdata(info));
                    }
                }
                else { // if old msg id, show message age
                    var msgage = Math.round(data.curts - data.msgts);
                    if ($(_this.info[platform]).find(".oldmsg").length <= 0) $(_this.info[platform]).append("<div class='error oldmsg'></div>");
                    $(_this.info[platform]).find(".oldmsg").html("<b style='color:rgb(197, 135, 0)'><small>LASTUPD:</small><br><big>"+msgage+"</big> s</b>");
                    //if (msgage > 3*60) $(_this.info[platform]).parent().css('background-color','#FFCCCC');
                    //else if (msgage > 30) $(_this.info[platform]).parent().css('background-color','#CCC');
                    if (msgage > 30) $(_this.info[platform]).parent().css('background-color','#CCC');
                    set_uncertainty(unclayer, data.pose);
                    //setTimeout(function(){$(_this.info[platform]).parent().css('background-color','white')}, 250);
                }
                var $flashupd = $(_this.info[platform]).parent().find('.heartbeat').show();
                setTimeout(function(){$flashupd.hide();},250)
                setTimeout(function(){_this.update_posetracker(tracklayer, unclayer, platform, url, interval, maxtracklen)},interval);
            },
            error : function (jqXHR, status, desc) {
                console.log("Cannot update position: "+platform,jqXHR);
                if ($(_this.info[platform]).find(".errmsg").length <= 0)
                    $(_this.info[platform]).html("<div class='error errmsg' data-count='0'></div>");
                $(_this.info[platform]).find(".errmsg").data('count',$(_this.info[platform]).find(".errmsg").data('count')+1);
                $(_this.info[platform]).find(".errmsg").html("Offline ("+$(_this.info[platform]).find(".errmsg").data('count')+")");
                $(_this.info[platform]).parent().css('background-color','#CCC');

                setTimeout(function(){_this.update_posetracker(tracklayer, unclayer, platform, url, interval, maxtracklen)},20000); // try again in 20 seconds if error
            }
        });
    }

    function set_pose(platform, tracklayer, unclayer, maxtracklen, pose) {
        var curpos = [];
        if ((pose.lat != NaN) && (pose.lon != NaN)) {
            curpos = new L.LatLng(pose.lat, pose.lon);

            // Add pose to track, but check if track is too long (to avoid memory/performance issues)
            if (_this.layers.overlays.hasOwnProperty(tracklayer)) {
                _this.layers.overlays[tracklayer].addLatLng(curpos);
                var tracklen = _this.layers.overlays[tracklayer].getLatLngs().length;
                if (tracklen > maxtracklen)
                    _this.layers.overlays[tracklayer].setLatLngs(_this.layers.overlays[tracklayer].getLatLngs().slice(tracklen - maxtracklen, tracklen));
            }

            set_uncertainty(platform, unclayer, pose);


            // Update marker / polygon position
            if (_this.layers.overlays[platform].hasOwnProperty("poly"))
                _this.layers.overlays[platform].poly.setLatLngHdg(pose.heading, curpos);//.bringToFront();
            else
                _this.layers.overlays[platform].setLatLng(curpos);
        }
    }


    function set_uncertainty(unclayer, pose) {
        var curpos = [];
        if ((pose.lat != NaN) && (pose.lon != NaN)) {
            var curpos = new L.LatLng(pose.lat, pose.lon);

            // set uncertainty circle
            var uncertainty = (pose.hasOwnProperty('uncertainty')) ? pose.uncertainty : 0.1;
            _this.layers.overlays[unclayer].setLatLng(curpos).setRadius(uncertainty);
        }
    }

    /**
     * Formats data for display.
     * @param data
     * @returns {string}
     */
    function formatdata (data, format) {
        if (typeof format === "undefined") format = "info"
        var datastr = "";
        var dataval = 0;
        if (format == "info") {
            for (var key in data) {
                dataval = data[key];
                //if (! isNaN(dataval)) dataval = dataval.toFixed(4);
                datastr += "<b>" + key + "</b>: " + dataval + "<br>";
            }
        }
        else if (format == "alerts") {
            for (var key in data) {
                if (data[key] == 1)
                    datastr += "<div class='alert alert-danger'>" + key + "</div>";
                    //datastr += "<div class='alert alert-success'>" + key + ": <b>OK</b></div>";
                else
                    datastr += "<div class='alert alert-success'>" + key + "</div>";
                    //datastr += "<div class='alert alert-danger'>" + key;// + ":<br><small>" + data[key] + "</small></div>";
            }
        }
        return datastr;
    }


    /**
     * Set map to full size
     */
    this.fullsize = function () {
        $("#"+this.mapid).css({width:$(window).width(),height:$(window).height()});
    }


    /**
     * Add controls for platform layer
     * @param platform
     * @param tracklayer
     * @param bgcol
     */
    this.add_platform_controls = function(platform, tracklayer, bgcol, dispoptions, allowcontrol) {
        var ctl = L.Control.extend({
            options: {
                position: 'bottomleft'
            },
            onAdd: function (map) {
                var ctldiv = L.DomUtil.create('div', 'platform-panel');
                $(ctldiv).prepend(
                    $("<b class='pname' style='cursor: pointer; cursor: hand;'><i class='fa fa-dot-circle-o platform-icon' style='color: "+bgcol+"'></i> "+platform+" <i class='fa fa-circle heartbeat'></i></b>")
                        .click(function(el){
                            if ($(_this.info[platform]).is(":visible")) {
                                $(this).css("color","#999");
                                $(_this.info[platform]).hide();
                            }
                            else {
                                $(this).css("color", "inherit");
                                $(_this.info[platform]).show();
                            }
                        })
                        .tooltip({title:"Show/hide info",trigger:"hover",container:"body"}),
                    $("<i class='fa fa-crosshairs platform-ctrl' id='snap-"+platform.replace(" ","_")+"'></i>")
                        .click(function(){_this.auto_extent(platform)})
                        .tooltip({title:"Keep in view",trigger:"hover",container:"body"})
                );
                if (dispoptions.showtrack)
                    $(ctldiv).prepend($("<i class='fa fa-tencent-weibo platform-ctrl'></i>")
                        .click(function(){_this.layers.overlays[tracklayer].setLatLngs([])})
                        .tooltip({title:"Clear track history",trigger:"hover",container:"body"})
                    );
                if (dispoptions.setwaypoint && allowadmin)
                    $(ctldiv).prepend($("<i class='fa fa-external-link-square platform-ctrl'></i>")
                        .click(function(){setwaypoint(platform)})
                        .tooltip({title:"Set waypoint",trigger:"hover",container:"body"})
                    );
                _this.info[platform] = $("<div class='info'></div>"); // empty div to update with platform info
                _this.info[platform].data('msgts',0); // initialise msgid
                $(ctldiv).append(_this.info[platform]);
                //$(ctldiv).css("background-color",bgcol);

                return ctldiv;
            }
        });
        this.map.addControl(new ctl());
    }


    /**
     * Add platform dashboard
     * @param platform
     */
    this.add_platform_dashboard = function(platform, bgcol, dispoptions) {
        if (dispoptions.showdash) {
            var ctl;
            ctl = L.Control.extend({
                options: {
                    position: 'bottomright'
                },
                onAdd: function (map) {
                    var ctldiv = L.DomUtil.create('div', 'platform-dash');
                    _this.dash[platform] = ctldiv;
                    var $hdg = $("<input class='knob hdg' data-width='150' data-cursor='10' data-bgColor='#ffffff' data-fgcolor='#428bca' data-thickness='.3'  value='0' data-min='0' data-max='360'>").knob({readOnly: true});
                    var $rol = $("<input class='knob rol' data-width='100' data-cursor='157' data-angleOffset='0' data-bgColor='#cccccc' data-fgcolor='#326594' data-thickness='0.9'  value='0' data-min='-180' data-max='180'>").knob({readOnly: true});
                    var $alerts = $("<div style='display: inline; float: left; width:100px'><div style='font-weight: bold; font-size: 16px'><i class='fa fa-dot-circle-o platform-icon' style='color: " + bgcol + "'></i> " + platform + "</div></div>");
                    var $dials = $("<div style='display: inline; float: left; width:150px'></div>");

                    $alerts.append(
                        $("<div class='platform-alerts'></div>"),
                        $("<div class='platform-stat'></div>")
                    );
                    if (dispoptions.showbatt) {
                        $alerts.append(
                            $("<div>Battery:</div>"),
                            $("<div class='progress' style='width: 90%;'><div class='progress-bar bat' role='progressbar' aria-valuenow='1' aria-valuemin='0' aria-valuemax='100' style='width: 1%;'>0%</div></div>")
                        );
                    }
                    if (dispoptions.showrph) {
                        $dials.append(
                            $("<div class='hdg-rol'></div>").append(
                                $("<div class='rol-canvas'></div>").html($rol),
                                $("<div class='hdg-canvas'></div>").html($hdg),
                                $("<div class='rph-info'>H:<br>P:<br>R:</div>")
                            )
                        );
                    }
                    $dials.append($("<div class='platform-pose'></div>"));

                    $(ctldiv).append($alerts, $dials).width(270);

                    // add dashboard icon to info panel
                    $(_this.info[platform]).parent().prepend(
                        $("<i class='fa fa-dashboard platform-ctrl active' id='dash-" + platform + "'></i>")
                            .click(function () {
                                $(_this.dash[platform]).toggle()
                                if ($(_this.dash[platform]).is(":visible")) $(this).addClass("active");
                                else $(this).removeClass("active");
                            })
                            .tooltip({title: "Show/hide dash", trigger: "hover", container: "body"})
                    );
                    return ctldiv;
                }
            });
            this.map.addControl(new ctl());
        }
    }

    /**
     * Add generic control to map.
     * @param html:     icon or html to insert into control
     * @param clickfnc: function to execute on click
     * @param title:    title to show on mouse over
     */
    this.add_control = function (html, clickfnc,title, position) {
        if (typeof position === 'undefined') position='topleft'; // set default position for control
        var tooltipos = (position.indexOf("right") > -1) ? "left" : "right"; // pick best tooltip location
        var ctl = L.Control.extend({
            options: {
                position: position
            },
            onAdd: function (map) {
                var ctldiv = L.DomUtil.create('div', 'map-control');
                $(ctldiv).html(html).click(clickfnc).tooltip({placement:tooltipos,container:"body",title:title,trigger:"hover"});
                return ctldiv;
            }
        });
        this.map.addControl(new ctl());
    }

    /**
     *
     * @param platform
     */
    function setwaypoint (platform) {
        bootbox.confirm("<div class='alert alert-danger'>You are about to set a waypoint for <b>"+platform+"</b>.</div>Click a location on the map to send the waypoint.", function(result){
            if (result)
                setTimeout(function(){
                    $(".leaflet-container").css("cursor","crosshair");
                    $(".leaflet-control-mouseposition").css("font-size","30px");
                    _this.map.on("click",function(e){
                        $.ajax({
                            dataType: "jsonp",
                            method: "POST",
                            url: "send_to_platform",
                            data: {platform: platform, lat:e.latlng.lat, lon: e.latlng.lng},
                            success: function (data) {
                                bootbox.alert(data.platform+"<br>Response: "+data.result);
                            },
                            error : function (jqXHR, status, desc) {
                                console.log("Cannot load mission: "+filepath,jqXHR);
                                setTimeout(function(){_this.get_mission(layer, filepath, url, usroptions, origin)},5000); // try again in 5 seconds
                            }
                        });
                        _this.map.off("click");
                        $(".leaflet-container").css("cursor","inherit");
                        $(".leaflet-control-mouseposition").css("font-size","inherit");
                    });
                },200);
        });
    }


    /**
     * Zoom to extent of track for a given platform.
     * Toggles tracking if 'forceautotrack' is not specified
     * Cancels any other existing tracker
     * @param platform
     * @param forceautotrack: true = enable autotrack, false = disable
     */
    this.auto_extent = function (platform, forceautotrack) {
        var autotrack =  (typeof forceautotrack === "undefined") ? ! _this.snap_to_track[platform] : forceautotrack;

        if (platform == "all" && autotrack == false) platform = Object.keys(_this.snap_to_track);
        else platform = [platform];

        $(platform).each(function(i,p) {
            _this.snap_to_track[p] = autotrack;
            if (autotrack) {
                $("#snap-" + p.replace(" ","_")).addClass("active");
                _this.autotrack_layer.addLayer(_this.layers.overlays[p]);
                //console.log(p, _this.layers.overlays)
            }
            else {
                $("#snap-" + p.replace(" ","_")).removeClass("active");
                _this.autotrack_layer.removeLayer(_this.layers.overlays[p]);
                _this.map.addLayer(_this.layers.overlays[p]);
            }
        });
    }


    /*
     * Helper functions for drawing the ship onto the map
     */

    function getShipPoly (size, shiplayer) {
        // Ship polygon is represented as a set of points in meters
        var length = size.length, width=size.width, offset=size.loffset;
        this.points = [[(-width / 2), 0.0-offset],
            [(width / 2), 0.0-offset],
            [(width / 2), (length - width-offset)],
            [0.0, (length-offset)],
            [(-width / 2), (length - width-offset)]
        ];
        this.shiplayer = shiplayer;

        function mdeglat(lat) {
            var latrad = lat/180*Math.PI;
            return 111132.09 - 566.05 * Math.cos(2.0*latrad) + 1.20 * Math.cos(4.0*latrad) - 0.002 * Math.cos(6.0*latrad) ;
        }

        function mdeglon(lat) {
            var latrad = lat/180*Math.PI;
            return 111415.13 * Math.cos(latrad) - 94.55 * Math.cos(3.0*latrad)  + 0.12 * Math.cos(5.0*latrad);
        }

        this.setLatLngHdg = function (heading, position) {
            var hdg = (((heading-90)/180)*Math.PI);

            // Rotate the points in the points array
            // Convert to LL and shift to the correct location
            var point = [],
                pointsLL = [];
            for (var i=0; i<this.points.length; i++) {
                point = [
                    (this.points[i][0] * Math.cos(hdg) - this.points[i][1] * Math.sin(hdg)) ,
                    (this.points[i][0] * Math.sin(hdg) + this.points[i][1] * Math.cos(hdg))
                ];
                pointsLL[i] = new L.LatLng(0,0);
                pointsLL[i].lat = (point[0] / mdeglat(position.lat) + position.lat);
                pointsLL[i].lng = (point[1] / mdeglon(position.lat) + position.lng);
            }
            this.shiplayer.setLatLngs(pointsLL);
            return this.shiplayer;
        }
    }
}
