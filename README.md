# Semestrální práce na AVG

**PROGRAM JE KE STÁHNUTÍ ZDE V [DEBUG VERZI](https://github.com/AshenCZ/avg-triangles/blob/master/download/debug_version.exe) A ZDE V [RELEASE VERZI](https://github.com/AshenCZ/avg-triangles/blob/master/download/release_version.exe)**

Úkolem bylo vytvořit program, který do 3D terénu (triangulovaného) promítne vzor zadaný jako seznam bodů a seznam hran. Protože promítnutí jsme omezili na osy X,Y a Z, pro snazší zobrazování v programu jsem rovnou použil 2D data - promítnutí podél jedné z os je jednoduché. Poté program pracuje v následujích krocích:

## Použití programu

### Slovní popis
* Uživatel vloží data počáteční sítě - pravým tlačítkem myši se vloží bod, vytvoří se nové trojúhelníky a provede se Delaunayova triangulace
* Uživatel může zkotrolovat korektnost inkrementálního triangulování zmáčknutím tlačítka "Triangulate", které zahodí všechna data o trojúhelnících a aktuální body znovu vytrianguluje inkrementálním algoritmem pro DT
* Zmáčknutím tlačítka **H** se uživateli zobrazí aktuální podoba, poloha, opakování a směr vzoru (červeně) a skryté hrany trojúhelníku, který byl vytvořen pro účely inkrementální DT
* Stisknutím tlačítka **F1** se vloží předem připravená malá síť pro testování
* Stisknutím tlačítka **I** uživatel může zadat nové parametry vzoru - nejdříve počet opakování (< 10), poté startovní bod (< (800,600)) a poté směr. Nastavení je přijato hláškou "Pattern set."
* Nové nastavení vzoru je možné opět zkontrolovat pomocí **H**.
* Tlačítkem **G** se poté vzor vyřeže do aktuální trojúhelníkové sítě. Zatím se neprovádí žádná triangulace. Důležité hrany vzoru jsou vybarvené fialově.
* Zmáčknutí klávesy **C** se poté provede jeden krok constrained Delaunayovy triangulace. Žádná z fialových hran nezmizí, ale jinak se celá síť dotrianguluje.
* Zmáčknutím tlačítka "Triangulate" je poté možno zjistit, zda se CDT liší od DT (pokud se triangulace změní po stisku tlačítka, byla CDT omezena nějakou fialovou hranou)
* Stisknutím klávesy **R** je možné celý program resetovat zpět do prázdného nastavení.
* Program je možno ukončit klávesou **Esc**

### Obrázky
#### Základní práce - vkládání nových bodů a zobrazení skrytých obejktů pomocí **H**
![](https://i.imgur.com/UN8ZZM5.gif)

#### Nové nastavení vzoru pomocí klávesy I a vložení testovací sítě pomocí klávesy F1
![](https://i.imgur.com/QWPRubb.gif)

#### Vyřezání hran vzoru do triangulace (klávesa G)
![](https://i.imgur.com/7mLotRn.gif)

#### Ukázka rozdílu DT (Triangulate) a CDT (klávesa C)
Na tomto .gifu je možné si všimnout, že provedením CDT až "nakonec" - program vypisuje "Not changed." v konzoli, pořád není dosaženo dokonalé Delaunayovy triangulace. Po stisknutí tlačítka "Triangulate" se triangulace změní a zároveň zmizí levé hrany horní části písmenka T.
![](https://i.imgur.com/ew8ooLu.gif)

#### Ukázka složitého vzoru, ve kterém CDT == DT
![](https://i.imgur.com/uhbU5Sf.gif)

## Implementace

Implementoval jsem několik algoritmů od začátku. Jediná knihovna, která byla použita je [SFML](http://sfml-dev.org/), což je grafická knihovna pro C++ pro zobrazení dat v okně.

### Inkrementální DT
Implementováno tak, jak jsme probírali na přednášce - vytvoří se trojúhelník obsahující celou obrazovku (a tedy všechny body, které uživatel je schopen vložit). Při vložení bodu se najde trojúhelník, ve kterém bod leží, trojúhelník se rozdělí na 3 nové a rekurzivně se pustí metoda `flip()`, která případně obrátí sousední trojúhelníky. Zde jsem narazil na nejasnost, protože metoda lépe funguje, pokud je implementována jako BFS, místo jako DFS. Při DFS se totiž občas snaží přehodit trojúhelník, který už neexistuje (protože byl přehozen v jiných větvích rekurze).

### Vyřezání vzoru do triangulace
Probíhá v několika částech:
1. Pomocí inkrementálního DT se vloží všechny body vzoru, ve všech polohách. Toto zajistí, že vyřezávání hran je trochu jednodušší.
2. Hrana po hraně se pustí metoda `insertEdge()`, která najde všechny trojúhelníky, které hrana protíná. Díky kroku (1) existují pouze dvě možnosti, jak může hrana trojúhelník protínat: vrchol-hrana nebo hrana-hrana (např. není možné, aby hrana začínala v bodě, který v triangulaci není, tudíž uvnitř trojúhelníku).
3. Pro všechny trojúhelníky, které hrana protíná se vytvoří nové 2(vrchol-hrana) nebo 3(hrana-hrana) trojúhelníky, a všechny důležité hrany se správně označí.

 Krok (3) je docela citlivý na přesnost a občas se stane, že body jsou moc blízko a začnou vznikat trojúhelníky s obsahem 0 (například vložením vzoru 2x za sebou bez změny počátku a směru). Program jsem protkal funkcí `assert()`, takže je velmi jednoduché zjistit, co se stalo.
 Krok (3) je také docela komplexní, protože je mnoho možností, jak hrana trojúhelník protíná a všechny se musí napsat do kódu samostatně. Ve složitějších situacích kdy se např. vzor protíná sem se sebou (díky dostatečně malému směru) vznikají dosti komplikované situace a označování hran jako důležitých není zcela jednoduché. Myslím si, že jsem program dostatečně otestovat a bude se chovat korektně, ale toto je místo, které je nejvíce problematické.
 
 ### Constrained Delaunay triangulace
 CDT je implementována jako jednoduchý flipping-edge algoritmus pro DT, pouze s tím rozdílem, že pokud je aktuální hrana důležitá, vůbec se o přehození nepřemýšlí a pokračuje se dále.
 
 Při stisknutí **C** se provede pouze jeden krok, protože jednak je hezké sledovat, které všechny trojúhelníky to přeskočí, a druhak protože se CDT občas zacyklí a přehazuje jednu hranu pořád tam a zpět. Toto chování mi není úplně jasné. Přestože v podmínce na přehození je ostrá nerovnost (bod musí ležet ostře uvnitř kružnice opsané), perfektně pravidelné lichoběžníky mají tendenci přehazovat úhlopříčku donekonečna.
