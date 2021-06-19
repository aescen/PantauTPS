<!DOCTYPE html>
<html>
	<head>
		<title>MONITORING TEMPAT PEMBUANGAN SAMPAH</title>
		<meta charset="UTF-8">
		<meta name="viewport" content="width=device-width, initial-scale=1">
		<link rel="shortcut icon" type="image/png" href="static/images/favicon.png"/>
		<link rel="stylesheet" href="static/styles/w3.css">
		<link rel="stylesheet" href="static/styles/css.css">
		<link rel="stylesheet" href="static/styles/font-awesome.css">
		<script src="static/scripts/canvasjs.min.js"></script>
		<script src="static/scripts/jquery.min.js"></script>
		<?php
			if( session_id() == ''){session_start();}
		?>
		<script languange="javascript">
			$(document).ready(function() {
				$("#feeds").load("feeds3.php");
				var refreshId = setInterval(function() {
					$("#feeds").load('feeds3.php');
				}, 2000);
				$.ajaxSetup({ cache: false });
			});
		</script>
	</head>
	
	<style>
		html,body,h1,h2,h3,h4,h5 {font-family: "Raleway", sans-serif}
	</style>
	
	<body class="w3-light-grey">
		<!-- Top container -->
		<div class="w3-bar w3-top w3-black w3-large" style="z-index:4">
		  <button type="button" class="w3-bar-item w3-button w3-hide-large w3-hover-none w3-hover-text-light-grey" onclick="w3_open();"><i class="fa fa-bars"></i>  Menu</button>
		  <span class="w3-bar-item w3-xlarge fa fa-cubes"> Monitoring Tempat Pembuangan Sampah</span>
		</div>

		<!-- Sidebar/menu -->
		<nav class="w3-sidebar w3-collapse w3-white w3-animate-left" style="z-index:3;width:300px;" id="mySidebar">
		  <!--<div class="w3-container w3-row">
			<div class="w3-col s4">
			  <img src="static/images/avatar2.png" class="w3-circle w3-margin-center" style="width:46px">
			</div>
			<div class="w3-col s8 w3-bar">
			  <span>Welcome, <strong>user</strong></span><br>
			  <a href="#" class="w3-bar-item w3-button"><i class="fa fa-envelope"></i></a>
			  <a href="#" class="w3-bar-item w3-button"><i class="fa fa-user"></i></a>
			  <a href="#" class="w3-bar-item w3-button"><i class="fa fa-cog"></i></a>
			</div>
		  </div>-->
		  <div class="w3-container w3-black">
			<h5>Dashboard</h5>
		  </div>
		  <div class="w3-bar-block">
			<a href="#" class="w3-bar-item w3-button w3-padding-16 w3-hide-large w3-grey w3-hover-black" onclick="w3_close()" title="close menu"><i class="fa fa-remove fa-fw"></i>  Close Menu</a>
			<a href="index.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 1</a>
			<a href="node2.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 2</a>
			<a href="node3.php" class="w3-bar-item w3-button w3-padding w3-blue"><i class="fa fa-dashboard fa-fw"></i>  Node 3</a>
			<a href="node4.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 4</a>
			<a href="logs.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Logs</a>
		  </div>
		</nav>

		<!-- Overlay effect when opening sidebar on small screens -->
		<div class="w3-overlay w3-hide-large w3-animate-opacity" onclick="w3_close()" style="cursor:pointer" title="close side menu" id="myOverlay"></div>

		<!-- !PAGE CONTENT! -->
		<div class="w3-main" style="margin-left:300px;margin-top:43px;">

		  <!-- Header -->
			<header class="w3-container">
				<h5><b><i class="fa fa-dashboard"></i> Status</b></h5>
			</header>
		  
		  <!-- Feeds panel -->
		  <div class="w3-container w3-row">
			<div class="w3-row w3-margin-top" name="feeds" id="feeds" ></div>
			<audio id='audioAlarm' src='./static/sounds/alarm-buzzer-short.mp3' type='audio/mpeg' preload='auto'></audio>
		  </div>

		  <!-- End page content -->
		</div>

		<script>
			var mySidebar = document.getElementById("mySidebar");
			var overlayBg = document.getElementById("myOverlay");
			function w3_open() {
			  if (mySidebar.style.display === 'block') {
				mySidebar.style.display = 'none';
				overlayBg.style.display = "none";
			  } else {
				mySidebar.style.display = 'block';
				overlayBg.style.display = "block";
			  }
			}
			function w3_close() {
			  mySidebar.style.display = "none";
			  overlayBg.style.display = "none";
			}
			
			var audioAlarm = new Audio('./static/sounds/alarm-buzzer-short.mp3');
			function play(){
				if(audioAlarm.duration != 0 && audioAlarm.currentTime != 0 && (audioAlarm.paused || audioAlarm.ended)){
					audioAlarm.setAttribute('src', './static/sounds/alarm-buzzer-short.mp3');
					//audioAlarm.autoplay = true;
					audioAlarm.loop = true;
					audioAlarm.play;
					document.body.appendChild(audioAlarm);
					var context = new webkitAudioContext();
					var analyser = context.createAnalyser();
					window.addEventListener('load', function(e) {
						var source = context.createMediaElementSource(audioAlarm);
						source.connect(analyser);
						analyser.connect(context.destination);
					}, false);
					console.log("Playing audio...");
					window.alert("Alarm ON!")
				} else {
					console.log("Audio already playing...");
				}
			}
			function stop(){
				audioAlarm.setAttribute('src', './static/sounds/alarm-buzzer-short.mp3');
				//audioAlarm.autoplay = false;
				audioAlarm.loop = false;
				audioAlarm.pause;
				audioAlarm.duration = 0;
				audioAlarm.currentTime = 0;
				console.log("Stopped");
			}
		</script>
	</body>
</html>