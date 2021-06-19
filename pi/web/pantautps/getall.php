<?php
  $mysqli = new mysqli("localhost", "rot", "", "wsn");
  $result = $mysqli->query("SELECT * FROM `Status` WHERE `Node` = 1 ");
  while($row = mysqli_fetch_assoc($result)){
    echo $row['suhu'].';'.$row['kelembaban'].';'.$row['moisture'].';';
  }
?>
