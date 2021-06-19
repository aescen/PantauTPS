
var routes = [
  {
    path: '/',
    url: './index.html',
  },
  {
   name: 'about',
   path: '/about/',
   url: './pages/about.html',
  },
  {
   path: '(.*)',
   url: './pages/404.html',
  },
];
