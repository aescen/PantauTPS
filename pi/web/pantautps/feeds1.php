<div id="chartContainer"></div>

<?php
	include("config.php");
	define("tCH4", 100);
	define("tCO2", 200);
	define("tNH3", 300);
	define("nodeId", 1);
	if( session_id() == ''){
		session_start();
	}
	if( !isset($_SESSION['play']) ){
			$_SESSION['play'] = false;
	}else{ $play = $_SESSION['play']; }
	
	if( !isset($_SESSION['alarm']) ){
			$_SESSION['alarm'] = false;
	}else{ $alarm = $_SESSION['alarm']; }
	
	if (!isset($play)){
		$play = $_SESSION['play'];
	}
	if($alarm == true){
		if($play == false){
			echo "<script>play();</script>";
			//echo "<script>console.log('Alarm:on');</script>";
			$play=true;
			$_SESSION['play'] = true;
		}else if($play == true){
			//echo "<script>console.log('Alarm:on:playing');</script>";
		}
	}else if($alarm == false){
		if($play == true){
			echo "<script>stop();</script>";
			//echo "<script>console.log('Alarm:off');</script>";
			$play=false;
			$_SESSION['play'] = false;
		}else if($play == false){
			//echo "<script>console.log('Alarm:off:stopped');</script>";
		}
	}
	
	$sqlCH4="SELECT * FROM (SELECT * FROM `pantautpslog` WHERE `Nama` LIKE 'ch4' AND `Node` = '" . nodeId . "' ORDER BY `Waktu` DESC LIMIT 12) aTable ORDER BY `Waktu` ASC";
	$sqlCO2="SELECT * FROM (SELECT * FROM `pantautpslog` WHERE `Nama` LIKE 'co2' AND `Node` = '" . nodeId . "' ORDER BY `Waktu` DESC LIMIT 12) aTable ORDER BY `Waktu` ASC";
	$sqlNH3="SELECT * FROM (SELECT * FROM `pantautpslog` WHERE `Nama` LIKE 'nh3' AND `Node` = '" . nodeId . "' ORDER BY `Waktu` DESC LIMIT 12) aTable ORDER BY `Waktu` ASC";
	$dataPointsCH4 = array();
	if ($resultCH4 = $conn->query($sqlCH4)) {
		while ($rowCH4 = $resultCH4->fetch_assoc()) {
			array_push($dataPointsCH4, array( "label" => $rowCH4["Waktu"], "y" => (float)$rowCH4["Nilai"] ));
			if((int)$rowCH4["Nilai"] >= tCH4){
				$_SESSION['btn'] = 'button-red';
				$_SESSION['setAlarm'] = true;
			}else{
				$_SESSION['btn'] = 'button-green';
				$_SESSION['setAlarm'] = false;
			}
		}
	}
	$dataPointsCO2 = array();
	if ($resultCO2 = $conn->query($sqlCO2)) {
		while ($rowCO2 = $resultCO2->fetch_assoc()) {
			array_push($dataPointsCO2, array( "label" => $rowCO2["Waktu"], "y" => (float)$rowCO2["Nilai"] ));
			if((int)$rowCO2["Nilai"] >= tCO2){
				$_SESSION['btn'] = 'button-red';
				$_SESSION['setAlarm'] = true;
			}else{
				$_SESSION['btn'] = 'button-green';
				$_SESSION['setAlarm'] = false;
			}
		}
	}
	$dataPointsNH3 = array();
	if ($resultNH3 = $conn->query($sqlNH3)) {
		while ($rowNH3 = $resultNH3->fetch_assoc()) {
			array_push($dataPointsNH3, array( "label" => $rowNH3["Waktu"], "y" => (float)$rowNH3["Nilai"] ));
			if((int)$rowNH3["Nilai"] >= tNH3){
				$_SESSION['btn'] = 'button-red';
				$_SESSION['setAlarm'] = true;
			}else{
				$_SESSION['btn'] = 'button-green';
				$_SESSION['setAlarm'] = false;
			}
		}
	}	
	if( ( isset($_SESSION['setAlarm']) ) && ( $_SESSION['setAlarm'] == true ) ){
		$_SESSION['alarm'] = true;
	}else{
		$_SESSION['alarm'] = false;
	}
?>

<script type="text/javascript">

    $(function () {
        var chart = new CanvasJS.Chart("chartContainer", {
            theme: "light2",
            zoomEnabled: true,
            title: {
                text: "Monitoring Tempat Pembuangan Sampah"
            },
            subtitles: [
                {
                    text: "Kadar gas: CH4;CO2;NH3"
				}
            ],
			axisY: {
				title: "Kadar",
				suffix: " ppm"
			},
            data: [
            {
                type: "line",
				name: "CH4",
				color: "#369EAD",
				showInLegend: true,
                dataPoints: <?php echo json_encode($dataPointsCH4); ?>
            },
			{
                type: "line",
				name: "CO2",
				color: "#C24642",
				showInLegend: true,
                dataPoints: <?php echo json_encode($dataPointsCO2); ?>
            },
			{
                type: "line",
				name: "NH3",
				color: "#66CC66",
				showInLegend: true,
                dataPoints: <?php echo json_encode($dataPointsNH3); ?>
            }
            ]
        });
        chart.render();
    });
</script>