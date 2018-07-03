% CWRU-Thesis.cls 
%
% Edited by Frank Ernst (Dr. rer. nat. habil., Professor).
%
% Based on (i) amsbook.cls and (ii) nuthesis.cls for writing doctoral
% dissertations at the Mathematics Department of Northwestern
% University by Miguel A. Lerma in the version of 06-05-09.

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\typeout{}
\ProvidesClass{CWRU-Thesis}[2006/05/09 v1.0 Frank Ernst]
\RequirePackage{ifthen}
\newboolean{help}
\setboolean{help}{false}
\DeclareOption{help}{\setboolean{help}{true}}
\PassOptionsToClass{oneside}{amsbook}
  \typeout{}
\DeclareOption*{
  \typeout{Pass to amsbook.cls: \CurrentOption}
  \PassOptionsToClass{\CurrentOption}{amsbook}}
\ProcessOptions\relax
\ifthenelse{\boolean{help}}
  {\typeout{}
   \typeout{*** CWRU-Thesis Help:}
   \typeout{*** Change linespacing with \protect\setstretch{real number}.}
   \typeout{*** Option reqno puts equation numbers on the right.}
   \typeout{*** Change hyperlink color with e.g. 
     \protect\definecolor{felinkcolor}{rgb}{0.5,0,0.5}}}
  {\typeout{}
   \typeout{*** Option "help" logs further information about the 
     CWRU-Thesis class.}}
\typeout{}
\LoadClassWithOptions{amsbook}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Choose between numbered and named citations:
\RequirePackage[comma, super, numbers, sort&compress, sectionbib]{natbib}
%\RequirePackage[comma, sort&compress, sectionbib]{natbib}

\RequirePackage{color}
\definecolor{felinkcolor}{rgb}{0,0,0}

\RequirePackage[bookmarks, bookmarksopen=true, plainpages=false,
pdfpagelabels, letterpaper, pdfpagelayout=SinglePage, colorlinks=true,
citecolor=felinkcolor, linkcolor=felinkcolor]{hyperref}

\RequirePackage{setspace}                        
\renewcommand{\onehalfspacing}{\setstretch{1.4}} 
\renewcommand{\doublespacing}{\setstretch{1.8}}  

% Redefine the headings pagestyle: amsbook.cls yields scriptsize and
% distinugishes odd from even pages, but we want normal size and the
% pagenumber always on the right (corresponding to odd pages).
\def\ps@headings{\ps@empty
  \def\@evenhead{\@oddhead}%
%     \setTrue{runhead}%
%     % Changed from scriptsize.
%     \normalfont\normalsize        
%     \rlap{\thepage}\hfil \leftmark{}{}\hfil}%
  \def\@oddhead{%
    \setTrue{runhead}%
    % Changed from scriptsize.
    \normalfont\normalsize \hfil
    \rightmark{}{}\hfill \llap{\thepage}}
  \let\@mkboth\markboth
  \def\partmark{\@secmark\markboth\partrunhead\partname}%
  \def\chaptermark{%
    \@secmark\markboth\chapterrunhead{}}%
  \def\sectionmark{%
    \@secmark\markright\sectionrunhead\sectionname} }

% Special page style for pages starting a new chapter. The page number
% appears on the top right, but the chapter title does not appear on
% the left.
\def\ps@newchapter{\ps@empty
  \def\@evenhead{\@oddhead}%
  \def\@oddhead{%
    % Changed from scriptsize.
    \normalfont\normalsize \hfil \llap{\thepage}}
  \let\@mkboth\markboth
  \def\partmark{}%
  \def\chaptermark{}%
  \def\sectionmark{}}

% Define the pagestyle for the frontmatter.
\def\ps@front{\ps@empty
  \def\@evenhead{\@oddhead}%
  \def\@oddhead{}%
  \def\@evenfoot{\@oddfoot}%
  \def\@oddfoot{%
    % Changed from scriptsize.
    \normalfont\normalsize \hfil
    \rightmark{}{}\hfill \llap{\thepage}}}

% Redefinition of \chapter* contributed by Oliver M. Haynold (5/19/2005)
% to fix a problem with bookmarks in PDF.

\newcommand\chapterX{\clearpage
%   \thispagestyle{plain}%
    \global\@topnum\z@
    \@afterindenttrue
    \secdef\@addchap\@saddchap}

% From the KOMA-Script scrbook.cls:
\newcommand\addchap{\clearpage
  \thispagestyle{myheadings}%
  \global\@topnum\z@
  \@afterindenttrue
  \secdef\@addchap\@saddchap}
\def\@addchap[#1]#2{\typeout{#2}
  \addcontentsline{toc}{chapter}{#1}%
  \if@twoside\@mkboth{#1}{}\else\@mkboth{}{#1}\fi
  \addtocontents{lof}{\protect\addvspace{10\p@}}%
  \addtocontents{lot}{\protect\addvspace{10\p@}}%
  \if@twocolumn
  \@topnewpage[\@makeschapterhead{#2}]%
  \else
  \@makeschapterhead{#2}%
  \@afterheading
  \fi}
\def\@saddchap#1{\@mkboth{}{}
  \if@twocolumn
  \@topnewpage[\@makeschapterhead{#1}]%
  \else
  \@makeschapterhead{#1}%
  \@afterheading
  \fi}

\def\l@figure{\@tocline{0}{3pt plus2pt}{0pt}{}{}}
\let\l@table=\l@figure

% Define page layout.
\voffset=-0.07in
\setlength{\textheight}{8.35in}
\setlength{\topmargin}{0in}
\setlength{\headsep}{0.5in}
\setlength{\footskip}{0.5in}
\setlength{\headheight}{12pt}
\setlength{\textwidth}{6in}
\setlength{\oddsidemargin}{0.5in}
\setlength{\evensidemargin}{0.5in}

% Format section numbering.
\renewcommand\thesection{\thechapter.\@arabic\c@section}
\renewcommand\thefigure {\thechapter.\arabic{figure}}
\renewcommand\thetable {\thechapter.\arabic{table}}

% Define layout of section headings.
\renewcommand{\section} {\@startsection{section}{1}
  {\z@}{-5ex\@plus -3ex}{.5\baselineskip}%
  {\normalfont\bfseries\large\fontfamily{pag}\selectfont}}

\renewcommand{\subsection} {\@startsection{subsection}{2}%
  {\z@}{-3.25ex\@plus -1ex \@minus -.2ex}{1.5ex \@plus .2ex}%
  {\normalfont\bfseries\fontfamily{pag}\selectfont}}

\renewcommand{\subsubsection} {\@startsection{subsubsection}{3}%
  {\z@}{-\z@}{-.5em}%
  {\normalfont\bfseries}}

% For mathemathicians.
\def\th@plain{%
  \thm@headfont{\bfseries}
  \let\thmhead\thmhead@plain \let\swappedhead\swappedhead@plain
  \thm@preskip.5\baselineskip\@plus.2\baselineskip
  \@minus.2\baselineskip
  \thm@postskip\thm@preskip
  \itshape}

\def\th@definition{%
  \thm@headfont{\bfseries}%
  \let\thmhead\thmhead@plain \let\swappedhead\swappedhead@plain
  \thm@preskip.5\baselineskip\@plus.2\baselineskip
  \@minus.2\baselineskip
  \thm@postskip\thm@preskip
  \upshape}

\def\th@remark{%
  \thm@headfont{\bfseries}%
  \let\thmhead\thmhead@plain \let\swappedhead\swappedhead@plain
  \thm@preskip.5\baselineskip\@plus.2\baselineskip
  \@minus.2\baselineskip
  \thm@postskip\thm@preskip
  \upshape}

% Redefine the font for proofname.
\renewcommand{\proofname}{\bfseries{Proof}}

\numberwithin{equation}{chapter}

% Redefine caption font (\scshape in amsbook.cls).
\def\@captionheadfont{\normalfont}  

% Define thesis prarameters
\def\@title{Title of dissertation required}
\def\degree#1{\def\d@gree{#1}}
\def\d@gree{Doctor of Philosophy} 
\def\adviser#1{\def\advis@r{Thesis Adviser:\ #1}}
\def\advis@r{}
\def\department#1{\def\dep@rtment{#1}}
\def\dep@rtment{Department Missing}
\def\campus#1{\def\c@mpus{#1}}
\def\c@mpus{Case Western Reserve University}

% Automatically determine graduation date.
\def\graduation@month{\ifcase\month\or
    January\or May\or May\or May\or May\or August\or August\or
    August\or January\or January\or January\or January\fi}
\def\graduation@year{\number\year}
\def\graduationmonth#1{\def\graduation@month{#1}}
\def\graduationyear#1{\def\graduation@year{#1}}
\def\@date{\graduation@month \graduation@year}

% Define special settings for thesis sections.
\def\frontmatter{%
  \def\ps@plain{\ps@front}
  \clearpage
  \pagestyle{front}
  \pagenumbering{roman}}
\def\mainmatter{%
  \pagestyle{myheadings}
  \def\ps@plain{\ps@headings}} 
\def\backmatter{\clearpage}
\def\titlepage{%
  \clearpage
  \thispagestyle{empty}\setcounter{page}{1}}
\def\endtitlepage{\newpage}
\def\copyrightpage{%
  \clearpage
  \thispagestyle{empty}
  \begin{center}
    \mbox{} 
    \vfill
    Copyright \copyright\ \graduation@year\ by \authors{}.\\
    All Rights Reserved.
  \end{center}
  \newpage
  \copyrightsignaturepage
  \thispagestyle{empty}}
\def\endcopyrightpage{\newpage}
\def\copyrightsignaturepage{%
  \thispagestyle{empty}
  \mbox{} 
  \begin{center}
    \vspace{3cm}
    \parbox{\hsize}{I grant to Case Western Reserve University the
      right to use this work, irrespective of any copyright, for the
      University's own purposes without cost to the University or to
      its students, agents, and employees. I further agree that the
      University may reproduce and provide single copies of the work,
      in any format other than in or from microforms, to the public
      for the cost of reproduction.}
    \vspace{1.5cm}
    \mbox{}
    \rule{\hsize}{0.5pt}
    \authors
  \end{center}}

\newcommand{\dedication}[1]{%
  \newpage
  \thispagestyle{empty}
  \renewcommand{\baselinestretch}{1}
  \begin{tabular}{p{\hsize}}
    \raggedleft
    \large
    \textit{#1}
  \end{tabular}
  \vfil}

\def\maketitle{%
  \par
  % This prevents figures from falling at the top of page 1.
  \@topnum\z@ 
  \begin{titlepage}
    \begin{center}
      \edef\uppercasetitle{\uppercase{\@title}}
      {\large\uppercasetitle}
      \vfill
      by\par
      \vspace{0.5cm}
      \ifx\@empty\authors
      Author's name required\par
      \else
      \edef\uppercaseauthors{\uppercase{\authors}}
      \uppercaseauthors\par
      \fi
      \vfill\vfill
      \vspace{0.5cm}
      Submitted in partial fulfillment of the requirements\par
      \vspace{0.5cm}
      For the degree of
      \d@gree{}\par
      \vfill
      \advis@r\par
      \vfill
      \dep@rtment\par
      \vspace*{6pt}
      CASE WESTERN RESERVE UNIVERSITY
      \vfill
      \vspace{0.5cm}
      \graduation@month{}, \graduation@year{}
    \end{center}
  \end{titlepage}}


% Generate page for signatures of committee members.

\newcommand{\signaturepage}{%
  \thispagestyle{empty}
  \begin{center}
    {\large\textbf{\@title}\\[0pt]
      Case Western Reserve University\\[-9pt]
      Case School of Graduate Studies\\[12pt]}
    We hereby approve the thesis\footnote{We certify that written approval has been obtained for any proprietary material contained therein.} of\par
    \textbf{\authors}\par
    for the degree of\par
    \textbf{\d@gree}
  \end{center}}

% Enter committee member signatures.
% Call: \sign[opt. Function, e.g. Adviser]{Name}{Chair or Member}{Department}
\newcommand{\sign}[4][]{
  \vspace{24pt}
  \noindent\textbf{#2}\\[-12pt]
  \rule{\hsize}{0.5pt}\\[-12pt]
  Committee #3\ifx#1\empty\else,\fi~#1\hfill Date: June 15th, 2018\\[-12pt]
  #4\par}


% Format abstract.
\renewcommand{\abstract}{%
  \chapter*{\abstractname}
  % \newpage
  \begin{center}
    {\large\@title{}}\par
    \bigskip
    Abstract\par
    by\par
    \bigskip
    \authors{}\par
  \end{center}
  \bigskip}
\def\abstractname{Abstract} 

% Format acknowledgements.
\newcommand{\acknowledgements}{\chapter*{\acknowledgementsname}}
\def\acknowledgementsname{Acknowledgements}

% Format preface.
\newcommand{\preface}{\chapter*{\prefacename}}
\def\prefacename{Preface}

% Format table of contents.
\renewcommand\contentsname{Table of Contents}

\newcommand{\vita}{\chapter*{\vitaname}}
\def\vitaname{Vita}

% Format bibliography. Normal size font, single line spacing, and
% double space between items.
%\def\bibname{References\\[-12pt]{\small\normalfont\textit{\ch@ptertitle}}}
\def\bibname{References}
\renewenvironment{thebibliography}[1]{%
  \@xp\chapter\@xp*\@xp{\bibname}%
  % \normalfont\footnotesize 
  \labelsep .5em\relax
  \renewcommand\theenumiv{\arabic{enumiv}}\let\p@enumiv\@empty
  \list{\@biblabel{\theenumiv}}{%
    \settowidth\itemsep\@mainsize
    \begin{singlespace}
      \settowidth\labelwidth{\@biblabel{#1}}%
      \leftmargin\labelwidth \advance\leftmargin\labelsep
      \usecounter{enumiv}}%
    \sloppy \clubpenalty\@M \widowpenalty\clubpenalty
    \sfcode`\.=\@m}
  {\def\@noitemerr{\@latex@warning{Empty `thebibliography' environment}}%
  \end{singlespace}
  \endlist}


% Format appendices.
\def\appendix{%
  \par
  \c@chapter\z@ \c@section\z@
  \def\thechapter{\@Alph\c@chapter}
  \def\appendixname{Appendix}
  \let\chaptername\appendixname
  %\def\thechapter{{}}
  \renewcommand\thesection{\@arabic\c@section}
  \def\@makechapterhead##1{\global\topskip 0pc\relax
    \begingroup
    \fontsize{\@xivpt}{18}\fontfamily{pag}\selectfont\bfseries\centering
    \chaptername\ \thechapter\\[6pt] ##1\par \endgroup
    \skip@34\p@ \advance\skip@-\normalbaselineskip
    \vskip\skip@ }
  \markboth{\textit{Appendix}}{\textit{Appendix}}}

\def\appendices{\par
  \c@chapter\z@ \c@section\z@
  \let\chaptername\appendixname
  \def\thechapter{\@Alph\c@chapter}}


% Format start of new chapter. Set afterindentfalse to suppress
% indentation of first line in parargraph after chapter headline.????

\def\chapter{%
  \clearpage\thispagestyle{plain}
  \global\@topnum\z@
  \@afterindenttrue \secdef\@chapter\@schapter}

\def\@makechapterhead#1{\global\topskip 5pc\relax%
  \def\ch@ptertitle{#1}
  \markright{\textit{\ch@ptertitle}\hfill}
  \thispagestyle{newchapter}
  \begingroup%
    \fontsize{\@xivpt}{18}\noindent%
    \huge{\fontfamily{pag}\selectfont\thechapter\quad#1}\par%
  \endgroup%
  \skip@120\p@\advance\skip@-\normalbaselineskip%
  \vskip\skip@\par}

\def\@makeschapterhead#1{\global\topskip 0pc\relax
  \begingroup
  \fontsize{\@xivpt}{18}\fontfamily{pag}\selectfont\bfseries\centering
  #1\par \endgroup
  \skip@34\p@ \advance\skip@-\normalbaselineskip
  \vskip\skip@ }

\def\@sect#1#2#3#4#5#6[#7]#8{%
  \edef\@toclevel{\ifnum#2=\@m 0\else\number#2\fi}%
  \ifnum #2<\c@secnumdepth\let\@secnumber\@empty
  \else \@xp\let\@xp\@secnumber\csname the#1\endcsname\fi
  \@tempskipa #5\relax
  \ifnum #2>\c@secnumdepth
  \let\@svsec\@empty
  \else
  \refstepcounter{#1}%
  \edef\@secnumpunct{%
    \ifdim\@tempskipa>\z@ % not a run-in section heading
    \@ifnotempty{#8}{\@nx\enspace}%
    \else
    \@ifempty{#8}{.}{\@nx\enspace}%
    \fi}
  \protected@edef\@svsec{%
    \ifnum#2<\@m
    \@ifundefined{#1name}{}{%
      \ignorespaces\csname #1name\endcsname\space}%
    \fi
    \@seccntformat{#1}}%
  \fi
  \ifdim \@tempskipa>\z@ 
  % This is not a run-in section heading.
  \begingroup #6\relax
  \@hangfrom{\hskip #3\relax\@svsec}{\interlinepenalty\@M #8\par}%
  \endgroup
  \csname #1mark\endcsname{#7}%
  \ifnum#2>\@m \else \@tocwrite{#1}{#8}\fi
  \else
  \def\@svsechd{%
    #6\hskip #3\@svsec
    \@ifnotempty{#8}{\ignorespaces#8\unskip
      \@addpunct.}%
    \ifnum#2>\@m \else \@tocwrite{#1}{#8}\fi}%
  \fi
  \global\@nobreaktrue
  \@xsect{#5}}

\def\@seccntformat#1{%
  \protect\textup{\protect\@secnumfont
    \csname the#1\endcsname\ 
    \protect\@secnumpunct}}

% Roman chapter number.
\renewcommand\thechapter{\arabic{chapter}}

% Limit numbering of headlines to 2 level.
\setcounter{secnumdepth}{2}

\doublespacing

\frontmatter 

\endinput

% End of file `CWRU-Thesis.cls'.
