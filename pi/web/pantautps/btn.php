<?php
	if( session_id() == ''){
		session_start();
	}
	echo "<button class='button button4 ".$_SESSION['btn']."'><strong>WARNING</strong></button>";
?>