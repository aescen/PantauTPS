<?php
	class Paginator {
	 
		private $_conn;
		private $_limit;
		private $_page;
		private $_query;
		private $_total;
			
		public function __construct( $conn, $query ) {     
			$this->_conn = $conn;
			$this->_query = $query;
		 
			$rs = $this->_conn->query( $this->_query );
			if ( !is_null($rs) || !empty($rs) || isset($rs) || $rs->num_rows > 0) {
				$this->_total = $rs->num_rows;
			}else{
				$this->_total = 0;
			}
		}
		
		public function getNumRows() {
			return $this->_total;
		}
		
		public function getLastPageNum(){
			$lastPage = 0;
			if( $this->_total > 0 && $this->_limit > 0){
				$lastPage = ceil( $this->_total / $this->_limit );
			}else{
				$lastPage = 0;
			}
			return $lastPage;
		}
		
		public function getData( $page = 1, $limit = 25 ) {     
			$this->_limit   = $limit;
			$this->_page    = $page;
		 
			if ( $this->_limit == 'all' ) {
				$query      = $this->_query;
			} else {
				$query      = $this->_query . " LIMIT " . ( ( $this->_page - 1 ) * $this->_limit ) . ", $this->_limit";
			}
			$rs             = $this->_conn->query( $query );
		 
			while ( $row = $rs->fetch_assoc() ) {
				$results[]  = $row;
			}
			
			$result         = new stdClass();
			$result->page   = $this->_page;
			$result->limit  = $this->_limit;
			$result->total  = $this->_total;
			$result->last   = ceil( $this->_total / $this->_limit );
			$result->data   = $results;
			
			return $result;
		}
		
		public function createLinks( $links, $list_class ) {
			if ( $this->_limit == 'all' ) {
				return '';
			}
		 
			$last       = ceil( $this->_total / $this->_limit );
		 
			$start      = ( ( $this->_page - $links ) > 0 ) ? $this->_page - $links : 1;
			$end        = ( ( $this->_page + $links ) < $last ) ? $this->_page + $links : $last;
		 
			$html       = '<div class="w3-center"><div class="w3-bar">';
		 
			$p      = ( $this->_page == 1 ) ? 1 : $this->_page - 1 ;
			$html       .= '<a class="'.$list_class.'" href="?limit=' . $this->_limit . '&page=' . $p . '">&laquo;</a>';
		 
			if ( $start > 1 ) {
				$html   .= '<a class="'.$list_class.'" href="?limit=' . $this->_limit . '&page=1">1</a>';
				$html   .= '<a class="'.$list_class.'" w3-disabled><span>...</span></a>';
			}
		 
			for ( $i = $start ; $i <= $end; $i++ ) {
				$class  = ( $this->_page == $i ) ? " w3-blue-grey" : "";
				$html   .= '<a class="'.$list_class.$class.'" href="?limit=' . $this->_limit . '&page=' . $i . '">' . $i . '</a>';
			}
		 
			if ( $end < $last ) {
				$html   .= '<a class="'.$list_class.'" w3-disabled><span>...</span></a>';
				$html   .= '<a class="'.$list_class.'" href="?limit=' . $this->_limit . '&page=' . $last . '">' . $last . '</a>';
			}
		 
			$p      = ( $this->_page == $last ) ? $last : $this->_page + 1 ;
			$html       .= '<a class="'.$list_class.'" href="?limit=' . $this->_limit . '&page=' . $p . '">&raquo;</a>';
		 
			$html       .= '</div></div>';
			return $html;
		}
	}
?>