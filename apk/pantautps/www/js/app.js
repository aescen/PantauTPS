var $$ = Dom7;
var $$data;
const $$tCH4 = 100.0;
const $$tCO2 = 200.0;
const $$tNH3 = 300.0;
var $$fbI;
var $$fbO;
var $$db;
var $$err = false;
var $$devReady = false;
var $$nid = 0;
var $$tmp = {
    "info": -1
  };

var app = new Framework7({
  root: '#app', // App root element
  id: 'id.ac.plnm.monitoring.tps', // App bundle ID
  name: 'Pantau TPS', // App name
  theme: 'md', // Automatic theme detection
  // App root data
  data() {
    return {
      foo: 'bar'
    };
  },
  // App root methods
  methods: {
    doSomething() {
      // ...
    }
  },
  // App routes
  routes: routes,
  // Input settings
  input: {
    scrollIntoViewOnFocus: Framework7.device.cordova && !Framework7.device.electron,
    scrollIntoViewCentered: Framework7.device.cordova && !Framework7.device.electron,
  },
  // Cordova Statusbar settings
  statusbar: {
    iosOverlaysWebView: true,
    androidOverlaysWebView: false,
  },
  on: {
    init: function () {
      var f7 = this;
      if (f7.device.cordova) {
        // Init cordova APIs (see cordova-app.js)
        cordovaApp.init(f7);
      }
    },
	pageInit: function () {
      console.log('Page initialized');
      $$devReady = true;
      refOnce();
    },
  },
});

window.onerror = function (msg, url, line) {
    $$err = true;
    app.preloader.hide();
    var msgs = "Message : " + msg + "<br>Line number : " + line + "<br>Url : " + url ;
    // Alert
    app.dialog.alert(msgs, 'Error', function(){
        if (typeof cordova !== 'undefined') {
            if (navigator.app) {
                navigator.app.exitApp();
            }
            else if (navigator.device) {
                navigator.device.exitApp();
            }
        } else {
            window.close();
        }
    });
}

function consoleToast(str){
    // Create bottom toast
    var toastBottom = app.toast.create({
      text: str,
      closeTimeout: 1500,
    });
    toastBottom.open();
}

//notifications
function addNotification(titlestr, textstr) {
    if (app.device.cordova) {
        cordovaApp.addNotification($$nid, titlestr, textstr);
        $$nid++;
        console.log("Notification added.");
        //consoleToast("Notification added.");
    }
    let loc = window.location.pathname;
    let dir = loc.substring(0, loc.lastIndexOf('/'));
    ////consoleToast('Loc:' + loc + ' Dir:' + dir);
    console.log('Loc:' + loc + ' Dir:' + dir);
}

// Initialize Firebase
function fbInit(){
    let config = {
        apiKey: "xxx",
        authDomain: "xxx",
        databaseURL: "xxx",
        projectId: "xxx",
        storageBucket: "xxx",
        messagingSenderId: "xxx",
        appId: "xxx"
    };
    // Initialize Firebase
    firebase.initializeApp(config);
    
    $$db = firebase.database();
    $$fbI = true;
    console.log("fbInit");
    
}

function fbOn(){
    /* obtain data when app is starting or data is updated */
    $$db.ref("/TPS/").on("value", function(snapshot) {
        $$data = snapshot.val();
        //console.log($$data);
		let info = -1;
		if($$data != null){
			if(typeof $$data.ch4 !== 'undefined' && typeof $$data.co2 !== 'undefined' && typeof $$data.nh3 !== 'undefined'){
				if(parseFloat($$data.ch4) >= $$tCH4){
					info = 1;
					document.getElementById('img-ch4').src = 'assets/ch4-red.png';
					document.getElementById('img-status').src = 'assets/indicator-red.png';
					document.getElementById('label-status').innerHTML = 'Peringatan!\nKadar CH4 tinggi!';
				} else if(parseFloat($$data.co2) >= $$tCO2){
					info = 1;
					document.getElementById('img-co2').src = 'assets/co2-red.png';
					document.getElementById('img-status').src = 'assets/indicator-red.png';
					document.getElementById('label-status').innerHTML = 'Peringatan!\nKadar CO2 tinggi!';
				} else if(parseFloat($$data.nh3) >= $$tNH3){
					info = 1;
					document.getElementById('img-nh3').src = 'assets/nh3-red.png';
					document.getElementById('img-status').src = 'assets/indicator-red.png';
					document.getElementById('label-status').innerHTML = 'Peringatan!\nKadar NH3 tinggi!';
				} else if(parseFloat($$data.ch4) < $$tCH4 && parseFloat($$data.co2) < $$tCO2 && parseFloat($$data.nh3) < $$tNH3){
					info = 0;
					document.getElementById('img-ch4').src = 'assets/ch4-green.png';
					document.getElementById('img-co2').src = 'assets/co2-green.png';
					document.getElementById('img-nh3').src = 'assets/nh3-green.png';
					document.getElementById('img-status').src = 'assets/indicator-green.png';
					document.getElementById('label-status').innerHTML = 'Normal/aman';
				}
				
				if($$tmp.info != info){
					if(info == 1){
						addNotification("Informasi TPS", "Peringatan!");
					} else if(info == 0){
						addNotification("Informasi TPS", "Aman.");
					}
					$$tmp.info = info;
				}
				
			} else {
				document.getElementById('img-ch4').src = 'assets/ch4-grey.png';
				document.getElementById('img-co2').src = 'assets/co2-grey.png';
				document.getElementById('img-nh3').src = 'assets/nh3-grey.png';
				document.getElementById('label-status').innerHTML = '...';
			}
		} else {
			document.getElementById('img-ch4').src = 'assets/ch4-grey.png';
			document.getElementById('img-co2').src = 'assets/co2-grey.png';
			document.getElementById('img-nh3').src = 'assets/nh3-grey.png';
			document.getElementById('label-status').innerHTML = '...';
			consoleToast("Data tidak siap!\n Cek koneksi.");
		}
		
		
        console.log("fbOn");
        
        app.preloader.hide();
        $$fbO = true;
        }, function (error) {
    });
}

/* obtain data once */
function refOnce(){
    if(!$$err && $$devReady){
        if(!$$fbI){
            fbInit();
            if(!$$fbO){
                fbOn();
            }
        } else {
            $$db.ref("/TPS/").once("value").then(
                function(snapshot) {
                    $$data = snapshot.val();
                    //console.log($$data);
                    let info = -1;
					if($$data != null){
						if(typeof $$data.ch4 !== 'undefined' && typeof $$data.co2 !== 'undefined' && typeof $$data.nh3 !== 'undefined'){
							if(parseFloat($$data.ch4) >= $$tCH4){
								info = 1;
								document.getElementById('img-ch4').src = 'assets/ch4-red.png';
								document.getElementById('img-status').src = 'assets/indicator-red.png';
								document.getElementById('label-status').innerHTML = 'Peringatan!\nKadar CH4 tinggi!';
							} else if(parseFloat($$data.co2) >= $$tCO2){
								info = 1;
								document.getElementById('img-co2').src = 'assets/co2-red.png';
								document.getElementById('img-status').src = 'assets/indicator-red.png';
								document.getElementById('label-status').innerHTML = 'Peringatan!\nKadar CO2 tinggi!';
							} else if(parseFloat($$data.nh3) >= $$tNH3){
								info = 1;
								document.getElementById('img-nh3').src = 'assets/nh3-red.png';
								document.getElementById('img-status').src = 'assets/indicator-red.png';
								document.getElementById('label-status').innerHTML = 'Peringatan!\nKadar NH3 tinggi!';
							} else if(parseFloat($$data.ch4) < $$tCH4 && parseFloat($$data.co2) < $$tCO2 && parseFloat($$data.nh3) < $$tNH3){
								info = 0;
								document.getElementById('img-ch4').src = 'assets/ch4-green.png';
								document.getElementById('img-co2').src = 'assets/co2-green.png';
								document.getElementById('img-nh3').src = 'assets/nh3-green.png';
								document.getElementById('img-status').src = 'assets/indicator-green.png';
								document.getElementById('label-status').innerHTML = 'Normal/aman';
							}
							
							if($$tmp.info != info){
								if(info == 1){
									addNotification("Informasi TPS", "Peringatan!");
								} else if(info == 0){
									addNotification("Informasi TPS", "Aman.");
								}
								$$tmp.info = info;
							}
							
						} else {
							document.getElementById('img-ch4').src = 'assets/ch4-grey.png';
							document.getElementById('img-co2').src = 'assets/co2-grey.png';
							document.getElementById('img-nh3').src = 'assets/nh3-grey.png';
							document.getElementById('label-status').innerHTML = '...';
						}
					} else {
						document.getElementById('img-ch4').src = 'assets/ch4-grey.png';
						document.getElementById('img-co2').src = 'assets/co2-grey.png';
						document.getElementById('img-nh3').src = 'assets/nh3-grey.png';
						document.getElementById('label-status').innerHTML = '...';
						consoleToast("Data tidak siap!\n Cek koneksi.");
					}
                    
                    console.log("fbOnce");
                    app.preloader.hide();
                }
            )
        }
    } else {
        console.log("Error/device not ready.");
        consoleToast("Error/device not ready.");
        app.preloader.hide();
    }
};

// Pull to refresh content
var $ptrContent = $$('.ptr-content');
$ptrContent.on('ptr:refresh', function (e) {
  setTimeout(function () {
        refOnce();
        console.log("Updated.");
        consoleToast("Updated.");
        app.ptr.done();
  }, 1000);
});